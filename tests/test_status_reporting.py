"""Exercise the production status encoder and transport under backpressure."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from test_distance_sensor import function

ROOT = Path(__file__).resolve().parents[1]


class StatusReporting(unittest.TestCase):
    def test_ros_single_shot_survives_cubemx_regeneration(self):
        source = (ROOT / 'Core/Src/main.c').read_text(encoding='utf-8')
        self.assertIn('hcan1.Init.AutoRetransmission = DISABLE;', function(source, 'MX_CAN1_Init'))
        self.assertIn('CAN1.NART=DISABLE', (ROOT / 'STM32_Motor_Controller.ioc').read_text())

    def test_status_types_mask_and_atomic_queue_acceptance(self):
        gcc = shutil.which('gcc')
        if not gcc:
            self.skipTest('Native GCC required')
        source = (ROOT / 'Core/Src/main.c').read_text(encoding='utf-8')
        encoder = '\n'.join(function(source, name) for name in (
            'CAN1_TrySend', 'CAN1_SendStatus', 'CAN1_SendAck', 'CAN1_SendDistance',
            'CAN1_SendSensorDiag', 'CAN1_SendStats', 'CAN1_SendEmergencyEvent', 'CAN1_SendButtonEvent'))
        checks = r'''
#define main existing_regression_main
#include "can_regression.c"
#include "distance_sensor.h"
#undef main
'''
        checks += encoder + r'''
int main(void)
{
    /* Reproduce a scheduler/mailbox stall while all four periodic producers
     * keep running. Only the newest snapshot for each stream may survive. */
    reset();
    motor_set_report_mask(7U);
    for (unsigned cycle = 0U; cycle < 50U; cycle++) {
        for (uint8_t motor = 0U; motor < MOTOR_COUNT; motor++) {
            motors[motor].current_velocity = (int32_t)(cycle * 10U + motor);
            CAN1_SendStatus(motor);
        }
    }
    assert(can_transport_get_failure_count(&hcan1) == 0U);
    assert(CAN1_SendAck(0x4321U, MOTOR_CMD_HEARTBEAT, 0U) == HAL_OK);
    drain();
    assert(sent_count[0] == 13U && sent[0][0].header.StdId == 0x102U);
    for (unsigned motor = 0U; motor < MOTOR_COUNT; motor++) {
        unsigned index = 1U + motor * 3U;
        assert(sent[0][index].data[0] == 1U && sent[0][index].data[1] == motor);
        assert(sent[0][index + 1U].data[0] == 2U);
        assert(sent[0][index + 2U].data[0] == 3U);
        unsigned velocity = (unsigned)sent[0][index + 2U].data[2] |
                            ((unsigned)sent[0][index + 2U].data[3] << 8);
        assert(velocity == 490U + motor);
    }
    reset();
    motor_set_report_mask(0U);
    assert(motor_get_report_mask() == 7U);
    motor_set_report_mask(3U);
    motors[0].position_feedback = -12345;
    CAN1_SendStatus(0U);
    drain();
    assert(sent_count[0] == 2U);
    assert(sent[0][0].header.StdId == 0x101U && sent[0][0].data[0] == 1U);
    assert(sent[0][1].data[0] == 2U);
    assert(sent[0][1].data[2] == 0xC7U && sent[0][1].data[3] == 0xCFU);
    assert(sent[0][1].data[4] == 0xFFU && sent[0][1].data[5] == 0xFFU);

    reset();
    motor_set_report_mask(3U);
    CAN_TxHeaderTypeDef filler = {.StdId = 0x103U, .DLC = 8U};
    for (unsigned i = 0U; i < CAN_TX_QUEUE_LENGTH - 1U; i++)
        assert(can_transport_send(&hcan1, &filler, (uint8_t[8]){0}) == HAL_OK);
    CAN1_SendStatus(0U);
    assert(can_transport_get_failure_count(&hcan1) == 0U);
    CAN_BusDiagnostics_t diag;
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.latest_backpressure == 1U);
    drain();
    assert(sent_count[0] == CAN_TX_QUEUE_LENGTH - 1U);
    for (unsigned i = 0U; i < sent_count[0]; i++)
        assert(sent[0][i].header.StdId == 0x103U);
    CAN1_SendStatus(0U);
    drain();
    assert(sent[0][63].data[0] == 1U && sent[0][64].data[0] == 2U);

    /* Exercise the actual ROS command parser and command queue. A BASIC-only
     * mask is deliberate; it never controls independent distance reporting. */
    CAN_RxHeaderTypeDef header = {.StdId = 0x100U, .IDE = CAN_ID_STD, .DLC = 8U};
    Motor_Command_t command;
    HAL_StatusTypeDef result;
    CAN1_RxCallback(header, (uint8_t[]){7,255,1,0,0,0,0,0});
    assert(motor_process_next_command(&command, &result) &&
           motor_take_command_result(&command, &result) && result == HAL_OK);
    assert(motor_get_report_mask() == 1U);
    sent_count[0] = 0U;
    CAN1_SendStatus(2U);
    drain();
    assert(sent_count[0] == 1U && sent[0][0].data[0] == 1U && sent[0][0].data[1] == 2U);
    CAN1_RxCallback(header, (uint8_t[]){7,255,7,0,0,0,0,0});
    assert(motor_process_next_command(&command, &result) &&
           motor_take_command_result(&command, &result) && result == HAL_OK);
    sent_count[0] = 0U;
    CAN1_SendStatus(2U);
    drain();
    assert(sent_count[0] == 3U);
    for (unsigned i = 0U; i < 3U; i++) {
        assert(sent[0][i].header.IDE == CAN_ID_STD && sent[0][i].header.DLC == 8U);
        assert(sent[0][i].data[0] == i + 1U && sent[0][i].data[1] == 2U);
    }
    motor_set_report_mask(15U);
    sent_count[0] = 0U;
    CAN1_SendStatus(3U);
    CAN1_SendStatus(MOTOR_COUNT);
    drain();
    assert(sent_count[0] == 5U);
    for (unsigned i = 0U; i < 5U; i++) assert(sent[0][i].data[0] == i + 1U);
    test_primask = 1U;
    CAN1_SendStatus(0U);
    assert(test_primask == 1U);
    test_primask = 0U;

    /* The production execution ACK must bypass a full telemetry queue. */
    reset();
    for (unsigned i = 0U; i < CAN_TX_QUEUE_LENGTH; i++)
        assert(can_transport_send(&hcan1, &filler, (uint8_t[8]){0}) == HAL_OK);
    assert(CAN1_SendAck(0x1234U, 9U, 0U) == HAL_OK);
    drain();
    assert(sent[0][0].header.StdId == 0x102U);
    assert(memcmp(sent[0][0].data, (uint8_t[]){0x12,0x34,9,0,0,0,0,0}, 8U) == 0);

    /* Production keys must distinguish sensors and statistic types, while
     * events and one-shot diagnoses remain separate even with identical data. */
    reset();
    distance_sensor_sample_t sample = {.sensor_id = 0U, .distance_mm = 500U};
    distance_sensor_diag_t diagnostic = {.sensor_id = 0U, .error_code = 3U};
    Motor_CommStats_t prior_stats;
    motor_get_comm_stats(&prior_stats);
    for (unsigned cycle = 0U; cycle < 100U; cycle++) {
        sample.sequence = (uint8_t)cycle;
        sample.sensor_id = 0U;
        sample.distance_mm = (uint16_t)(500U + cycle);
        assert(CAN1_SendDistance(&sample) == HAL_OK);
        sample.sensor_id = 1U;
        assert(CAN1_SendDistance(&sample) == HAL_OK);
        motor_record_rx_result(1U);
        CAN1_SendStats();
    }
    assert(CAN1_SendSensorDiag(&diagnostic) == HAL_OK);
    assert(CAN1_SendSensorDiag(&diagnostic) == HAL_OK);
    CAN1_SendEmergencyEvent(0U, 0x80U, 0);
    CAN1_SendEmergencyEvent(0U, 0x80U, 0);
    CAN1_SendButtonEvent(1U, 0U, 77U);
    CAN1_SendButtonEvent(1U, 0U, 77U);
    drain();
    assert(sent_count[0] == 11U);
    assert(sent[0][0].data[0] == 6U && sent[0][1].data[0] == 6U);
    assert(sent[0][2].header.StdId == ROS_CAN_BUTTON_EVENT_ID);
    assert(sent[0][3].header.StdId == ROS_CAN_BUTTON_EVENT_ID);
    for (unsigned sensor = 0U; sensor < 2U; sensor++) {
        assert(sent[0][4U + sensor].header.StdId == ROS_CAN_DISTANCE_ID);
        assert(sent[0][4U + sensor].data[1] == sensor);
        assert(sent[0][4U + sensor].data[7] == 99U);
        assert(sent[0][4U + sensor].data[2] == (599U & 255U));
    }
    for (unsigned type = 1U; type <= 3U; type++)
        assert(sent[0][5U + type].header.StdId == ROS_CAN_STATS_ID && sent[0][5U + type].data[0] == type);
    assert(sent[0][6].data[1] == (uint8_t)(prior_stats.rx_count + 100U));
    assert(sent[0][9].header.StdId == ROS_CAN_SENSOR_DIAG_ID);
    assert(sent[0][10].header.StdId == ROS_CAN_SENSOR_DIAG_ID);

    /* 60 simulated seconds at the actual default production rates: 600 motor
     * snapshots/s, 120 ranges/s, 37.5 stats/s, 50 native replies/s and 10
     * heartbeat ACKs/s. Hardware is unavailable for 10 ms out of each 100 ms.
     * Check bounded latency as well as capacity; this is not an on-board test. */
    reset();
    unsigned ack_count = 0U, native_count = 0U;
    unsigned max_ack_delay = 0U, max_native_delay = 0U;
    unsigned first_seq = motor_allocate_cmd_seq();
    motor_set_report_mask(7U);
    for (tick = 0U; tick < 60000U; tick++) {
        if (tick % 20U == 0U) {
            for (uint8_t motor = 0U; motor < MOTOR_COUNT; motor++) CAN1_SendStatus(motor);
            CAN_TxHeaderTypeDef raw = {.IDE = CAN_ID_EXT,
                .ExtId = ((tick / 20U % 4U) + 1U) << 8, .DLC = 5U};
            uint8_t payload[8] = {0x35U, (uint8_t)tick, (uint8_t)(tick >> 8), 0U, 0x6BU};
            assert(can_transport_forward(&hcan1, &raw, payload) == HAL_OK);
        }
        if (tick % 50U == 0U) {
            for (uint8_t sensor = 0U; sensor < DISTANCE_SENSOR_COUNT; sensor++) {
                sample.sensor_id = sensor;
                sample.sequence = (uint8_t)(tick / 50U);
                assert(CAN1_SendDistance(&sample) == HAL_OK);
            }
        }
        if (tick % 80U == 0U) CAN1_SendStats();
        if (tick % 100U == 0U) {
            CAN1_RxCallback(header, (uint8_t[]){9U,255U,0U,0U,0U,0U,0U,0U});
            assert(motor_process_next_command(&command, &result) &&
           motor_take_command_result(&command, &result) && result == HAL_OK);
            assert(CAN1_SendAck(command.seq, command.cmd, 0U) == HAL_OK);
        }
        free_mailboxes[0] = tick % 100U < 10U ? 0U : 3U;
        can_transport_service();
        for (unsigned n = 0U; n < sent_count[0]; n++) {
            CAN_TxFrame_t *frame = &sent[0][n];
            if (frame->header.IDE == CAN_ID_STD && frame->header.StdId == ROS_CAN_ACK_ID) {
                unsigned seq = ((unsigned)frame->data[0] << 8) | frame->data[1];
                assert(seq - first_seq == ++ack_count && frame->data[2] == 9U && frame->data[3] == 0U);
                unsigned delay = tick - (ack_count - 1U) * 100U;
                if (delay > max_ack_delay) max_ack_delay = delay;
            } else if (frame->header.IDE == CAN_ID_EXT) {
                unsigned enqueue_tick = frame->data[1] | ((unsigned)frame->data[2] << 8);
                unsigned delay = tick - enqueue_tick;
                assert(enqueue_tick == native_count * 20U);
                native_count++;
                if (delay > max_native_delay) max_native_delay = delay;
            }
        }
        sent_count[0] = 0U;
        assert(can_transport_get_failure_count(&hcan1) == 0U);
    }
    assert(ack_count == 600U && native_count == 3000U);
    assert(max_ack_delay <= 12U && max_native_delay < 100U);
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.latest_replaced > 0U && diag.latest_backpressure == 0U);
    assert(diag.tx_high_water <= CAN_TX_QUEUE_LENGTH - CAN_TX_RESERVED_SLOTS);
    printf("PASS 60s simulated default load: ACK=%u/native=%u, max ACK/native delay=%u/%u ms, high-water=%u, failures=0\n",
           ack_count, native_count, max_ack_delay, max_native_delay, (unsigned)diag.tx_high_water);
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix='status-reporting-') as folder:
            tmp = Path(folder)
            c = tmp / 'check.c'
            c.write_text(checks, encoding='utf-8')
            exe = tmp / 'check.exe'
            sources = ['can_transport', 'zdt_can_driver', 'motor_control', 'can_protocol',
                       'zdt_status', 'stm32f4xx_it']
            subprocess.run([gcc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-include', str(ROOT / 'tests/support/hal_stub.h'),
                            '-I', str(ROOT / 'tests/support'), '-I', str(ROOT / 'tests'),
                            '-I', str(ROOT / 'Core/Inc'), str(c),
                            *[str(ROOT / 'Core/Src' / (name + '.c')) for name in sources],
                            '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
