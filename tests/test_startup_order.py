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
    def test_start_buttons_match_cubemx(self):
        header=(ROOT/'Core/Inc/main.h').read_text(encoding='utf-8')
        ioc=(ROOT/'STM32_Motor_Controller.ioc').read_text(encoding='utf-8')
        self.assertRegex(header,r'#define START_BUTTON_Pin\s+GPIO_PIN_1')
        self.assertRegex(header,r'#define ESTOP_BUTTON_Pin\s+GPIO_PIN_2')
        self.assertIn('PC1.GPIO_Label=START_BUTTON',ioc)
        self.assertIn('PC2.GPIO_Label=ESTOP_BUTTON',ioc)

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
