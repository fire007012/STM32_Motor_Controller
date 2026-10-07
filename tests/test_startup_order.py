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
    return text[start:text.index('void SystemClock_Config(void)', start)]


class StartupOrder(unittest.TestCase):
    def test_hal_delay_sensor_init_precedes_rtos_objects(self):
        main = main_body()
        self.assertEqual(main.count('distance_sensor_init()'), 1)
        sensor = main.index('distance_sensor_init()')
        # Queue/task creation may mask the low-priority HAL tick before scheduling.
        for operation in ['osKernelInitialize()', 'motor_control_init()', 'osThreadNew(',
                          'motor_enable_all()', 'osKernelStart()']:
            with self.subTest(operation=operation):
                self.assertLess(sensor, main.index(operation))

    def test_peripheral_and_servo_setup_stays_before_sensor_init(self):
        main = main_body()
        sensor = main.index('distance_sensor_init()')
        for operation in ['HAL_Init()', 'SystemClock_Config()', 'MX_GPIO_Init()',
                          'MX_CAN1_Init()', 'MX_CAN2_Init()', 'Servo_Init()', 'CAN_Filter_Config()']:
            with self.subTest(operation=operation):
                self.assertLess(main.index(operation), sensor)

    def test_can_and_transport_remain_after_rtos_queue_setup(self):
        main = main_body()
        self.assertLess(main.index('can_transport_init('), main.index('CAN_ServiceStartup()'))
        self.assertLess(main.index('motor_control_init()'), main.index('CAN_ServiceStartup()'))
        self.assertLess(main.index('CAN_ServiceStartup()'), main.index('osKernelStart()'))


if __name__ == '__main__':
    unittest.main()
