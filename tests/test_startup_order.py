"""Source-order guard for HAL tick initialization before FreeRTOS masks IRQs.

This checks startup ordering, not real hardware timing or scheduler execution.
"""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]


def main_body():
    text = (ROOT / 'Core/Src/main.c').read_text(encoding='utf-8')
    text = re.sub(r'/\*.*?\*/|//[^\n]*', '', text, flags=re.S)
    start = text.index('int main(void)')
    return text[start:]


class StartupOrder(unittest.TestCase):
    def test_start_button_matches_cubemx(self):
        header=(ROOT/'Core/Inc/main.h').read_text(encoding='utf-8')
        ioc=(ROOT/'STM32_Motor_Controller.ioc').read_text(encoding='utf-8')
        self.assertRegex(header,r'#define START_BUTTON_Pin\s+GPIO_PIN_1')
        self.assertIn('PC1.GPIO_Label=START_BUTTON',ioc)
        self.assertNotIn('ESTOP_BUTTON', header)
        self.assertNotIn('PC2.', ioc)
        self.assertNotRegex(ioc, r'(?m)^Mcu\.Pin\d+=PC2$')

    def test_arm_key_scan_is_outside_tim6_interrupt(self):
        text=(ROOT/'Core/Src/main.c').read_text(encoding='utf-8')
        task=text[text.index('void StartMotorControlTask(void *argument)\n{'):text.index('void StartStatusTask(void *argument)\n{')]
        timer=text[text.index('void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)\n{'):text.index('void Error_Handler(void)\n{')]
        self.assertIn('Servo_PollButtons();',task)
        self.assertNotIn('Servo_PollButtons();',timer)
        self.assertNotIn('ArmServo_PollButtons(',timer)
        self.assertIn('motor_safety_poll();',task)

    def test_wkup_claims_local_sg90_control_before_starting_sequence(self):
        text=(ROOT/'Core/Src/main.c').read_text(encoding='utf-8')
        body=text[text.index('static uint8_t Servo_DemoClaimLocal(void)\n{'):text.index('static uint8_t Servo_DemoSetBoth(int16_t angle_deg)\n{')]
        self.assertIn('servo_remote_owned = 0U;',body)
        self.assertIn('servo_return_pending = 0U;',body)
        wkup=text[text.index('static void WkupDemo_Service(void)\n{'):text.index('/* USER CODE END 4 */')]
        self.assertIn('Servo_DemoClaimLocal() && ArmServo_DemoSetAngle',wkup)

    def test_wkup_dispenser_sequence_is_ordered_and_nonblocking(self):
        config=(ROOT/'Core/Inc/arm_servo_config.h').read_text(encoding='utf-8')
        source=(ROOT/'Core/Src/main.c').read_text(encoding='utf-8')
        self.assertRegex(config,r'ARM_SERVO_WK_PLACE_DEG\s+30')
        self.assertRegex(config,r'ARM_SERVO_WK_HOLD_MS\s+1000U')
        self.assertRegex(config,r'ARM_SERVO_WK_SERVO_ANGLE\s+0')
        self.assertRegex(config,r'ARM_SERVO_WK_SERVO_RETURN_ANGLE\s+180')
        self.assertRegex(config,r'ARM_SERVO_WK_SERVO_HOLD_MS\s+3000U')
        self.assertRegex(config,r'ARM_SERVO_WK_SERVO_RETURN_MS\s+1000U')
        body=source[source.index('static void WkupDemo_Service(void)\n{'):source.index('/* USER CODE END 4 */')]
        steps=['case WKUP_DEMO_ARM_FORWARD:','case WKUP_DEMO_SERVO_OPEN:','case WKUP_DEMO_SERVO_CLOSE:','case WKUP_DEMO_ARM_RETURN:','ArmServo_DemoEnd()']
        positions=[body.index(step) for step in steps[:-1]]+[body.rindex(steps[-1])]
        self.assertEqual(positions,sorted(positions))
        self.assertNotIn('HAL_Delay(',body)
        self.assertNotIn('osDelay(',body)

    def test_hal_delay_sensor_init_precedes_rtos_objects(self):
        main = main_body()
        self.assertEqual(main.count('distance_sensor_init()'), 1)
        sensor = main.index('distance_sensor_init()')
        # Queue/task creation may mask the low-priority HAL tick before scheduling.
        for operation in ['osKernelInitialize()', 'motor_control_init()', 'osThreadNew(',
                          'osKernelStart()']:
            with self.subTest(operation=operation):
                self.assertLess(sensor, main.index(operation))

    def test_peripheral_and_servo_setup_stays_before_sensor_init(self):
        main = main_body()
        sensor = main.index('distance_sensor_init()')
        for operation in ['HAL_Init()', 'SystemClock_Config()', 'MX_GPIO_Init()',
                          'MX_CAN1_Init()', 'MX_CAN2_Init()', 'Servo_Init()', 'CAN_Filter_Config()']:
            with self.subTest(operation=operation):
                self.assertLess(main.index(operation), sensor)

    def test_remote_arm_init_is_not_disabled_by_local_key_selection(self):
        main = main_body()
        arm = main.index('ArmServo_Init()')
        self.assertLess(main.index('Servo_Init()'), arm)
        self.assertLess(arm, main.index('distance_sensor_init()'))
        self.assertNotIn('ARM_SERVO_KEY_TEST_ENABLE', main)

    def test_can_and_transport_remain_after_rtos_queue_setup(self):
        main = main_body()
        self.assertLess(main.index('can_transport_init('), main.index('osKernelStart()'))
        self.assertLess(main.index('motor_control_init()'), main.index('osKernelStart()'))
        self.assertGreater(main.index('CAN_ServiceStartup()'), main.index('osKernelStart()'))
        self.assertGreater(main.index('motor_enable_all()'), main.index('osKernelStart()'))


if __name__ == '__main__':
    unittest.main()
