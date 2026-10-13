"""Exercise production sensor bring-up, sampling, reporting and stop decisions.

Register-level I2C is replaced with a mock bus; this is not a hardware test.
Selected production functions are compiled together, as in test_sg90_can.py.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    import re
    match = re.search(r'^(?:static )?(?:void|uint\d+_t|HAL_StatusTypeDef)\s+' +
                      name + r'\([^;]*?\)\s*\{', source, re.M)
    if not match:
        raise ValueError(name)
    brace = source.index('{', match.start())
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


class DistanceSensorBehavior(unittest.TestCase):
    def test_five_sensor_bus_reporting_and_emergency_scope(self):
        gcc = shutil.which('gcc')
        if not gcc:
            self.skipTest('Native GCC required')
        source = (ROOT / 'Core/Src/distance_sensor.c').read_text(encoding='utf-8')
        declarations = source[:source.index('static uint16_t read_be16')]
        functions = '\n'.join(function(source, name) for name in (
            'read_be16', 'sensor_set_xshut', 'sensor_mark_error', 'vl53_init_one',
            'median_filter', 'vl53_process_ready_sample', 'distance_sensor_init',
            'distance_sensor_poll', 'distance_sensor_take_updated',
            'distance_sensor_take_diagnostic', 'distance_sensor_take_emergency'))
        main = (ROOT / 'Core/Src/main.c').read_text(encoding='utf-8')
        functions += '\n' + function(main, 'CAN1_SendDistance')
        with tempfile.TemporaryDirectory(prefix='distance-sensors-') as folder:
            tmp = Path(folder)
            shutil.copyfile(ROOT / 'Core/Inc/distance_sensor.h', tmp / 'distance_sensor.h')
            shutil.copyfile(ROOT / 'Core/Inc/can_protocol.h', tmp / 'can_protocol.h')
            (tmp / 'main.h').write_text(r'''
#ifndef TEST_DISTANCE_MAIN_H
#define TEST_DISTANCE_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum {HAL_OK, HAL_ERROR} HAL_StatusTypeDef;
typedef enum {GPIO_PIN_RESET, GPIO_PIN_SET} GPIO_PinState;
typedef struct {unsigned unused;} GPIO_TypeDef;
extern GPIO_TypeDef gpioe;
#define GPIOE (&gpioe)
#define GPIO_PIN_0 1U
#define GPIO_PIN_1 2U
#define GPIO_PIN_2 4U
#define GPIO_PIN_5 32U
#define GPIO_PIN_6 64U
#define CAN_ID_STD 0U
#define CAN_RTR_DATA 0U
#define DISABLE 0U
typedef struct {uint32_t StdId,ExtId,IDE,RTR,DLC,TransmitGlobalTime;} CAN_TxHeaderTypeDef;
typedef struct {unsigned unused;} CAN_RxHeaderTypeDef;
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
void HAL_GPIO_WritePin(GPIO_TypeDef *port,uint16_t pin,GPIO_PinState state);
#endif
''', encoding='utf-8')
            c = tmp / 'check.c'
            c.write_text('#include <assert.h>\n#include "can_protocol.h"\n' + declarations +
                         (ROOT / 'tests/distance_sensor_mocks.inc').read_text(encoding='utf-8') +
                         functions +
                         (ROOT / 'tests/distance_sensor_checks.inc').read_text(encoding='utf-8'),
                         encoding='utf-8')
            exe = tmp / 'check.exe'
            subprocess.run([gcc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-I', str(tmp), str(c), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
