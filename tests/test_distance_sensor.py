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
    def test_actual_ranging_configuration_and_data_ready_polarity(self):
        gcc = shutil.which('gcc')
        if not gcc:
            self.skipTest('Native GCC required')
        source = (ROOT / 'Core/Src/distance_sensor.c').read_text(encoding='utf-8')
        declarations = source[:source.index('static vl53l1x_sensor_t sensors')]
        functions = '\n'.join(function(source, name) for name in (
            'vl53_encode_timeout', 'vl53_macro_period_us', 'vl53_timeout_us_to_mclks',
            'vl53_set_timing_budget', 'vl53_apply_ranging_config', 'vl53_start_continuous'))
        checks = r'''
static uint8_t registers[512];
static uint16_t failing_register = 0xFFFFU;
static HAL_StatusTypeDef vl53_read_u8(uint8_t addr, uint16_t reg, uint8_t *value)
{ assert(addr == 0x30U); *value = registers[reg]; return reg == failing_register ? HAL_ERROR : HAL_OK; }
static HAL_StatusTypeDef vl53_read_u16(uint8_t addr, uint16_t reg, uint16_t *value)
{ assert(addr == 0x30U); *value = ((uint16_t)registers[reg] << 8) | registers[reg+1U]; return HAL_OK; }
static HAL_StatusTypeDef vl53_write_u8(uint8_t addr, uint16_t reg, uint8_t value)
{ assert(addr == 0x30U); if (reg == failing_register) return HAL_ERROR; registers[reg] = value; return HAL_OK; }
static HAL_StatusTypeDef vl53_write_u16(uint8_t addr, uint16_t reg, uint16_t value)
{ assert(addr == 0x30U); registers[reg] = (uint8_t)(value >> 8); registers[reg+1U] = (uint8_t)value; return HAL_OK; }
static HAL_StatusTypeDef vl53_write_u32(uint8_t addr, uint16_t reg, uint32_t value)
{ assert(addr == 0x30U); for (unsigned i=0U; i<4U; i++) registers[reg+i]=(uint8_t)(value >> (24U-8U*i)); return HAL_OK; }
'''
        checks += functions + r'''
int main(void)
{
    vl53l1x_sensor_t sensor = {.address=0x30U, .fast_osc_frequency=10000U, .osc_calibrate_val=200U};
    registers[VL53L1X_GPIO_MUX_REG] = 1U; /* Initially active high. */
    registers[VL53L1X_INTERRUPT_CONFIG_REG] = 0U; /* Initially threshold mode. */
    assert(vl53_apply_ranging_config(&sensor) == HAL_OK);
    assert(registers[VL53L1X_GPIO_MUX_REG] == 0x11U);
    assert(registers[VL53L1X_INTERRUPT_CONFIG_REG] == 0x20U);
    assert(registers[VL53L1X_EXTSUP_CONFIG_REG] & 1U);
    assert(registers[0x82U] == 0U && registers[0x81U] == 0x8BU);
    assert(vl53_start_continuous(&sensor) == HAL_OK);
    assert(registers[VL53L1X_MODE_START_REG] == 0x40U);
    assert(registers[VL53L1X_INTERRUPT_CLEAR_REG] == 1U);
    uint32_t period = 0U;
    for (unsigned i=0U; i<4U; i++) period=(period<<8)|registers[VL53L1X_INTERMEASUREMENT_REG+i];
    assert(period == 50U * 200U);
    failing_register = VL53L1X_GPIO_MUX_REG;
    assert(vl53_apply_ranging_config(&sensor) == HAL_ERROR);
    failing_register = VL53L1X_INTERRUPT_CONFIG_REG;
    assert(vl53_apply_ranging_config(&sensor) == HAL_ERROR);
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix='distance-config-') as folder:
            tmp = Path(folder)
            c = tmp / 'check.c'
            c.write_text('#include <assert.h>\ntypedef struct {unsigned unused;} GPIO_TypeDef;\n' +
                         declarations + checks, encoding='utf-8')
            exe = tmp / 'check.exe'
            subprocess.run([gcc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-include', str(ROOT / 'tests/support/hal_stub.h'),
                            '-I', str(ROOT / 'Core/Inc'), str(c), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_reports_retry_after_transport_queue_exhaustion(self):
        gcc = shutil.which('gcc')
        if not gcc:
            self.skipTest('Native GCC required')
        source = (ROOT / 'Core/Src/distance_sensor.c').read_text(encoding='utf-8')
        declarations = source[:source.index('static uint16_t read_be16')]
        functions = '\n'.join(function(source, name) for name in (
            'read_be16', 'sensor_set_xshut', 'sensor_mark_error', 'vl53_configure_one',
            'vl53_wait_for_boot', 'vl53_init_one',
            'distance_sensor_recover',
            'vl53_process_ready_sample', 'distance_sensor_init',
            'distance_sensor_poll', 'distance_sensor_take_updated',
            'distance_sensor_take_diagnostic'))
        main = (ROOT / 'Core/Src/main.c').read_text(encoding='utf-8')
        functions += '\n' + '\n'.join(function(main, name) for name in (
            'CAN1_TrySend', 'CAN1_SendDistance', 'CAN1_SendSensorDiag', 'StartSensorTask'))
        mocks = (ROOT / 'tests/distance_sensor_mocks.inc').read_text(encoding='utf-8')
        mocks = mocks.replace(function(mocks, 'CAN1_TrySend'), '')
        checks = (ROOT / 'tests/distance_sensor_reporting_checks.inc').read_text(encoding='utf-8')
        with tempfile.TemporaryDirectory(prefix='distance-report-retry-') as folder:
            tmp = Path(folder)
            stub = (ROOT / 'tests/support/hal_stub.h').read_text(encoding='utf-8')
            stub += r'''
typedef enum {GPIO_PIN_RESET, GPIO_PIN_SET} GPIO_PinState;
typedef struct {unsigned unused;} GPIO_TypeDef;
extern GPIO_TypeDef gpioe;
#define GPIOE (&gpioe)
#define GPIO_PIN_0 1U
#define GPIO_PIN_1 2U
#define GPIO_PIN_2 4U
#define GPIO_PIN_5 32U
#define GPIO_PIN_6 64U
#define GPIO_PIN_7 128U
void HAL_Delay(uint32_t ms);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
void osDelay(uint32_t ms);
HAL_StatusTypeDef motor_stop_all(void);

'''
            (tmp / 'test_hal.h').write_text(stub, encoding='utf-8')
            c = tmp / 'check.c'
            # Keep the actual task loop alive across retries; osDelay drives the
            # mock bus and ends the test after recovery and duplicate checks.
            c.write_text('#include <assert.h>\n#include <stdio.h>\n#include <setjmp.h>\n'
                         '#include "can_protocol.h"\n#include "can_transport.h"\n' +
                         declarations + mocks + checks[:checks.index('/* TASK FUNCTIONS */')] +
                         functions + checks[checks.index('/* TASK FUNCTIONS */'):], encoding='utf-8')
            exe = tmp / 'check.exe'
            subprocess.run([gcc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-include', str(tmp / 'test_hal.h'), '-I', str(ROOT / 'Core/Inc'),
                            str(c), str(ROOT / 'Core/Src/can_transport.c'), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_six_sensor_bus_reporting_and_emergency_scope(self):
        gcc = shutil.which('gcc')
        if not gcc:
            self.skipTest('Native GCC required')
        source = (ROOT / 'Core/Src/distance_sensor.c').read_text(encoding='utf-8')
        declarations = source[:source.index('static uint16_t read_be16')]
        functions = '\n'.join(function(source, name) for name in (
            'read_be16', 'sensor_set_xshut', 'sensor_mark_error', 'vl53_configure_one',
            'vl53_wait_for_boot', 'vl53_init_one',
            'distance_sensor_recover',
            'vl53_process_ready_sample', 'distance_sensor_init',
            'distance_sensor_poll', 'distance_sensor_take_updated',
            'distance_sensor_take_diagnostic'))
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
#define GPIO_PIN_7 128U
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
                            '-I', str(tmp), '-I', str(ROOT / 'Core/Inc'), str(c), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
