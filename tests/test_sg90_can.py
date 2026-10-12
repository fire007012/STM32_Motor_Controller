"""Exercise actual SG90 command parser from main.c with native fake timers."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class PositionServoCAN(unittest.TestCase):
    def test_ack_commands_checksum_ranges_and_local_ownership(self):
        gcc = shutil.which('gcc')
        if not gcc:
            self.skipTest('Native GCC required')
        source = (ROOT / 'Core/Src/main.c').read_text(encoding='utf-8')
        macros = '\n'.join(line for line in source.splitlines()
                           if line.startswith('#define SERVO_') and not line.rstrip().endswith('\\'))
        functions = source[source.index('static uint32_t Servo_AngleToPulse('):
                           source.index('static void Servo_PollButtons(void)\n{')]
        stub = r"""
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#define ARM_SERVO_KEY_TEST_ENABLE 0U
#define POSITION_SERVO_CMD 0x22U
#define HAL_OK 0
#define HAL_ERROR 1
#define TIM_CHANNEL_3 3
#define __HAL_TIM_SET_COMPARE(t,c,v) ((void)(c),(t)->pulse=(v))
typedef int HAL_StatusTypeDef;
typedef struct {uint32_t pulse;} TIM_HandleTypeDef;
static TIM_HandleTypeDef htim3,htim4;
static uint8_t servo_return_pending,servo_return_angle,servo_return_mask,servo_initialized,servo_remote_owned;
static uint32_t tick,servo_return_tick,primask;
static uint32_t HAL_GetTick(void){return tick;}
static uint32_t __get_PRIMASK(void){return primask;}
static void __disable_irq(void){primask=1;}
static void __enable_irq(void){primask=0;}
static int HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,uint32_t c){(void)t;(void)c;return HAL_OK;}
"""
        checks = r"""
static void sum(uint8_t d[8]){d[7]=0;for(int i=0;i<7;i++)d[7]+=d[i];}
int main(void){
 assert(Servo_Init()==HAL_OK);assert(htim3.pulse==250 && htim4.pulse==250);
 uint8_t d[8]={0x22,1,0,0,0,0,42,0};sum(d);
 assert(Servo_HandlePositionCanCommand(d)==0);assert(htim3.pulse==55 && htim4.pulse==250);
 assert(servo_remote_owned);Servo_SetLocalBoth(180);assert(htim3.pulse==55);
 Servo_ReleaseLocalControl();Servo_SetLocalBoth(180);assert(htim3.pulse==250);
 d[1]=2;sum(d);assert(Servo_HandlePositionCanCommand(d)==0);assert(htim4.pulse==55);
 d[1]=3;d[2]=90;d[3]=181;sum(d);assert(Servo_HandlePositionCanCommand(d)==2);
 assert(htim3.pulse==250 && htim4.pulse==55);
 d[3]=180;sum(d);assert(Servo_HandlePositionCanCommand(d)==0);
 assert(htim3.pulse==153 && htim4.pulse==250);
 d[7]++;assert(Servo_HandlePositionCanCommand(d)==1);
 d[1]=4;sum(d);assert(Servo_HandlePositionCanCommand(d)==1);
 d[1]=0;sum(d);assert(Servo_HandlePositionCanCommand(d)==1);
 d[1]=3;d[4]=1;sum(d);assert(Servo_HandlePositionCanCommand(d)==1);
 d[4]=0;sum(d);servo_initialized=0;assert(Servo_HandlePositionCanCommand(d)==3);
 Servo_ReleaseLocalControl();Servo_AcquireRemoteControl();assert(servo_remote_owned);
 Servo_ReleaseLocalControl();primask=1;Servo_SetLocalBoth(180);assert(primask==1);
 assert(Servo_HandlePositionCanCommand(NULL)==1);
 return 0;
}
"""
        with tempfile.TemporaryDirectory(prefix='sg90-can-test-') as folder:
            directory = Path(folder)
            c = directory / 'test.c'
            executable = directory / ('test.exe' if __import__('sys').platform=='win32' else 'test')
            c.write_text(stub+'\n'+macros+'\n'+functions+'\n'+checks,encoding='utf-8')
            subprocess.run([gcc,'-std=c11','-Wall','-Wextra','-Werror',str(c),'-o',str(executable)],check=True)
            subprocess.run([str(executable)],check=True)

if __name__=='__main__':
    unittest.main()
