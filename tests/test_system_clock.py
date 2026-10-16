"""Run the production clock setup with HAL recorders for both clock sources."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from test_distance_sensor import function

ROOT = Path(__file__).resolve().parents[1]


class SystemClock(unittest.TestCase):
    def test_hse_default_and_opt_in_hsi_preserve_peripheral_rates(self):
        gcc = shutil.which('gcc')
        if not gcc:
            self.skipTest('Native GCC required')
        source = function((ROOT / 'Core/Src/main.c').read_text(encoding='utf-8'),
                          'SystemClock_Config')
        scaffold = r'''
#include <assert.h>
#define RCC_OSCILLATORTYPE_HSE 1
#define RCC_OSCILLATORTYPE_HSI 2
#define RCC_HSE_ON 1
#define RCC_HSI_ON 1
#define RCC_HSICALIBRATION_DEFAULT 16
#define RCC_PLL_ON 1
#define RCC_PLL_OFF 0
#define RCC_PLLSOURCE_HSE 1
#define RCC_PLLSOURCE_HSI 2
#define RCC_PLLP_DIV2 2
#define RCC_CLOCKTYPE_HCLK 1
#define RCC_CLOCKTYPE_SYSCLK 2
#define RCC_CLOCKTYPE_PCLK1 4
#define RCC_CLOCKTYPE_PCLK2 8
#define RCC_SYSCLKSOURCE_PLLCLK 2
#define RCC_SYSCLKSOURCE_HSE 1
#define RCC_SYSCLK_DIV1 1
#define RCC_HCLK_DIV2 2
#define PWR_REGULATOR_VOLTAGE_SCALE1 1
#define FLASH_LATENCY_2 2
#define HAL_OK 0
#define __HAL_RCC_PWR_CLK_ENABLE() ((void)0)
#define __HAL_PWR_VOLTAGESCALING_CONFIG(x) ((void)(x))
typedef struct {
    unsigned OscillatorType, HSEState, HSIState, HSICalibrationValue;
    struct { unsigned PLLState, PLLSource, PLLM, PLLN, PLLP, PLLQ; } PLL;
} RCC_OscInitTypeDef;
typedef struct {
    unsigned ClockType, SYSCLKSource, AHBCLKDivider, APB1CLKDivider, APB2CLKDivider;
} RCC_ClkInitTypeDef;
static RCC_OscInitTypeDef osc;
static RCC_ClkInitTypeDef clk;
static unsigned calls;
static int HAL_RCC_OscConfig(RCC_OscInitTypeDef *value) {
    assert(calls++ == 0); osc = *value; return HAL_OK;
}
static int HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *value, unsigned latency) {
    assert(calls++ == 1); assert(latency == FLASH_LATENCY_2); clk = *value; return HAL_OK;
}
static void Error_Handler(void) { assert(0); }
'''
        checks = r'''
int main(void) {
    SystemClock_Config();
    assert(calls == 2);
    unsigned input;
#if defined(STM32_DIAGNOSTIC_HSE_DIRECT) && STM32_DIAGNOSTIC_HSE_DIRECT
    assert(osc.OscillatorType == RCC_OSCILLATORTYPE_HSE);
    assert(osc.HSEState == RCC_HSE_ON);
    assert(osc.PLL.PLLState == RCC_PLL_OFF);
    assert(clk.SYSCLKSource == RCC_SYSCLKSOURCE_HSE);
    input = 8000000;
#elif defined(STM32_DIAGNOSTIC_HSI_CLOCK) && STM32_DIAGNOSTIC_HSI_CLOCK
    assert(osc.OscillatorType == RCC_OSCILLATORTYPE_HSI);
    assert(osc.HSIState == RCC_HSI_ON && osc.HSEState == 0);
    assert(osc.HSICalibrationValue == RCC_HSICALIBRATION_DEFAULT);
    assert(osc.PLL.PLLSource == RCC_PLLSOURCE_HSI);
    input = 16000000;
#else
    assert(osc.OscillatorType == RCC_OSCILLATORTYPE_HSE);
    assert(osc.HSEState == RCC_HSE_ON);
    assert(osc.PLL.PLLSource == RCC_PLLSOURCE_HSE);
    input = 8000000;
#endif
    assert(clk.ClockType == 15);
#if defined(STM32_DIAGNOSTIC_HSE_DIRECT) && STM32_DIAGNOSTIC_HSE_DIRECT
    unsigned sys = input;
#else
    assert(osc.PLL.PLLState == RCC_PLL_ON);
    assert(osc.PLL.PLLQ == 3);
    assert(clk.SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK);
    unsigned sys = input / osc.PLL.PLLM * osc.PLL.PLLN / osc.PLL.PLLP;
#endif
    unsigned hclk = sys / clk.AHBCLKDivider;
    unsigned apb1 = hclk / clk.APB1CLKDivider;
    unsigned apb2 = hclk / clk.APB2CLKDivider;
#if defined(STM32_DIAGNOSTIC_HSE_DIRECT) && STM32_DIAGNOSTIC_HSE_DIRECT
    assert(sys == 8000000 && hclk == 8000000);
    assert(apb1 == 4000000 && apb2 == 4000000);
#else
    assert(sys == 72000000 && hclk == 72000000);
    assert(apb1 == 36000000 && apb2 == 36000000);
    assert(apb1 / (6 * (1 + 9 + 2)) == 500000);
    assert(apb1 * 2 / 720 / 2000 == 50);
#endif
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix='system-clock-') as folder:
            tmp = Path(folder)
            c = tmp / 'clock.c'
            c.write_text(scaffold + source + checks, encoding='utf-8')
            exe = tmp / 'clock.exe'
            for flags in ([], ['-DSTM32_DIAGNOSTIC_HSI_CLOCK=0'],
                          ['-DSTM32_DIAGNOSTIC_HSI_CLOCK=1'],
                          ['-DSTM32_DIAGNOSTIC_HSE_DIRECT=1']):
                with self.subTest(flags=flags):
                    subprocess.run([gcc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                                    *flags, str(c), '-o', str(exe)], check=True)
                    subprocess.run([str(exe)], check=True)
