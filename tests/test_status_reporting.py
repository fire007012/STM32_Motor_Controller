"""Exercise the production status encoder and transport under backpressure."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from test_distance_sensor import function

ROOT = Path(__file__).resolve().parents[1]


class StatusReporting(unittest.TestCase):
    def test_status_types_mask_and_atomic_queue_acceptance(self):
        gcc = shutil.which('gcc')
        if not gcc:
            self.skipTest('Native GCC required')
        source = (ROOT / 'Core/Src/main.c').read_text(encoding='utf-8')
        encoder = function(source, 'CAN1_SendStatus')
        checks = r'''
#define main existing_regression_main
#include "can_regression.c"
#undef main
'''
        checks += encoder + r'''
int main(void)
{
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
    assert(can_transport_get_failure_count(&hcan1) == 1U);
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
    assert(motor_process_next_command(&command, &result) && result == HAL_OK);
    assert(motor_get_report_mask() == 1U);
    sent_count[0] = 0U;
    CAN1_SendStatus(2U);
    drain();
    assert(sent_count[0] == 1U && sent[0][0].data[0] == 1U && sent[0][0].data[1] == 2U);
    CAN1_RxCallback(header, (uint8_t[]){7,255,7,0,0,0,0,0});
    assert(motor_process_next_command(&command, &result) && result == HAL_OK);
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
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix='status-reporting-') as folder:
            tmp = Path(folder)
            c = tmp / 'check.c'
            c.write_text(checks, encoding='utf-8')
            exe = tmp / 'check.exe'
            sources = ['can_transport', 'zdt_can_driver', 'motor_control', 'can_protocol',
                       'zdt_status', 'stm32f4xx_it', 'chassis_collision']
            subprocess.run([gcc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-DCHASSIS_KINEMATICS_HEADER="chassis_test_config.h"',
                            '-include', str(ROOT / 'tests/support/hal_stub.h'),
                            '-I', str(ROOT / 'tests/support'), '-I', str(ROOT / 'tests'),
                            '-I', str(ROOT / 'Core/Inc'), str(c),
                            *[str(ROOT / 'Core/Src' / (name + '.c')) for name in sources],
                            '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
