#include "arm_servo.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
RCC_TypeDef fake_rcc;
uint32_t fake_compare[4];
uint32_t fake_primask;
unsigned fake_clock_flags;
static uint32_t tick,clock_hz=36000000U;
static uint8_t k0,k1;
static unsigned configured,started,stopped;
static int fail_start=-1, fail_config=-1;
static TIM_HandleTypeDef saved_timer;
uint32_t HAL_RCC_GetPCLK2Freq(void){return clock_hz;}
uint32_t HAL_GetTick(void){return tick;}
uint8_t HAL_GPIO_ReadPin(void *port,uint32_t pin){assert(port==GPIOE);return !(pin==GPIO_PIN_4?k0:k1);}
void HAL_GPIO_Init(void *port,GPIO_InitTypeDef *gpio){
 assert(port==GPIOC);assert(gpio->Pin==(GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9));
 assert(gpio->Alternate==GPIO_AF3_TIM8 && gpio->Mode==GPIO_MODE_AF_PP);
}
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *timer){assert(timer->Instance==TIM8);saved_timer=*timer;return HAL_OK;}
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *timer,TIM_OC_InitTypeDef *oc,uint32_t ch){
 (void)timer;assert(oc->OCMode==TIM_OCMODE_PWM1);assert(oc->Pulse==0);configured|=1U<<(ch/4);
 return (int)(ch/4)==fail_config?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer,uint32_t ch){
 (void)timer;if((int)(ch/4)==fail_start)return HAL_ERROR;
 assert(ch<8);assert(fake_compare[ch/4]==1500);started|=1U<<(ch/4);return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *timer,uint32_t event){(void)timer;assert(event==TIM_EVENTSOURCE_UPDATE);return HAL_OK;}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer,uint32_t ch){(void)timer;stopped|=1U<<(ch/4);started&=~(1U<<(ch/4));return HAL_OK;}
static void poll_ms(unsigned n){while(n--){++tick;ArmServo_PollButtons(k0,k1);}}
static void press(unsigned key){if(key==0)k0=1;else k1=1;poll_ms(30);k0=k1=0;poll_ms(30);}
static void angles(int16_t angle){assert(ArmServo_GetTargetAngle(1)==angle);assert(ArmServo_GetTargetAngle(2)==angle);assert(fake_compare[2]==0 && fake_compare[3]==0);}
static void prepare(void){configured=started=stopped=fake_clock_flags=0;memset(fake_compare,0,sizeof(fake_compare));k0=k1=0;fail_start=fail_config=-1;fake_rcc.CFGR=RCC_CFGR_PPRE2;clock_hz=36000000U;}
static void command(uint8_t opcode,int16_t first10,int16_t second10,uint8_t mask,uint8_t result){
 uint8_t data[8]={opcode,mask,(uint8_t)first10,(uint8_t)((uint16_t)first10>>8),
    (uint8_t)second10,(uint8_t)((uint16_t)second10>>8),42,0};
 for(unsigned i=0;i<7;i++)data[7]+=data[i];
 assert(ArmServo_HandleCanCommand(data)==result);
}
int main(void){
 prepare();assert(ArmServo_Init()==HAL_OK);angles(0);
 assert(configured==15 && started==3 && fake_clock_flags==3);
 assert(saved_timer.Init.Prescaler==71 && saved_timer.Init.Period==19999);
 press(0);angles(5);
 long delta=(2000*5+ARM_SERVO_1_TRAVEL_DEG/2)/ARM_SERVO_1_TRAVEL_DEG;
 assert(fake_compare[0]==(uint32_t)(1500+ARM_SERVO_1_DIRECTION*delta));
 assert(fake_compare[1]==(uint32_t)(1500+ARM_SERVO_2_DIRECTION*delta));
 k0=1;poll_ms(3000);angles(10);k0=0;poll_ms(30);
 press(1);angles(5);press(1);angles(0);
 k0=1;poll_ms(5);k0=0;poll_ms(30);angles(0);
 for(unsigned i=0;i<30;i++)press(0);angles(60);
 for(unsigned i=0;i<30;i++)press(1);angles(-60);
 for(unsigned i=0;i<12;i++)press(0);angles(0);
 k0=k1=1;poll_ms(100);angles(0);k1=0;poll_ms(100);angles(0);
 k0=0;poll_ms(30);press(0);angles(5);
 tick=UINT32_MAX-10U;press(1);angles(0);
 prepare();k0=1;assert(ArmServo_Init()==HAL_OK);poll_ms(1000);angles(0);
 k0=0;poll_ms(30);press(0);angles(5);
 prepare();assert(ArmServo_Init()==HAL_OK);
 command(0x21,150,-200,3,0);assert(ArmServo_GetTargetAngle(1)==15 && ArmServo_GetTargetAngle(2)==-20);
 press(0);assert(ArmServo_GetTargetAngle(1)==15 && ArmServo_GetTargetAngle(2)==-20);
 command(0x21,0,700,3,2);assert(ArmServo_GetTargetAngle(1)==15 && ArmServo_GetTargetAngle(2)==-20);
 command(0x21,0,0,4,1);command(0x21,0,0,0,1);command(0x99,0,0,3,1);
 command(0x23,0,0,0,0);press(1);assert(ArmServo_GetTargetAngle(1)==15);
 command(0x21,0,0,3,0);angles(0);command(0x24,0,0,0,0);press(0);angles(5);
 command(0x21,0,0,3,0);command(0x24,0,0,0,0);angles(0);
 ArmServo_PollDemo(1,0);tick+=50;ArmServo_PollDemo(1,0);angles(0); /* held at boot */
 ArmServo_PollDemo(0,0);tick+=25;ArmServo_PollDemo(0,0);
 ArmServo_PollDemo(1,0);tick+=25;ArmServo_PollDemo(1,0);angles(15);
 tick+=ARM_SERVO_WK_MOVE_MS+ARM_SERVO_WK_HOLD_MS-1;ArmServo_PollDemo(1,0);angles(15);
 tick+=1;ArmServo_PollDemo(1,0);angles(0);tick+=5000;ArmServo_PollDemo(1,0);angles(0);
 ArmServo_PollDemo(0,0);tick+=25;ArmServo_PollDemo(0,0);
 ArmServo_PollDemo(1,0);tick+=25;ArmServo_PollDemo(1,0);angles(15);
 command(0x21,50,50,3,0);tick+=5000;ArmServo_PollDemo(0,0);angles(5); /* remote wins */
 command(0x21,0,0,3,0);command(0x24,0,0,0,0);angles(0);
 tick+=25;ArmServo_PollDemo(0,0);ArmServo_PollDemo(1,1);tick+=25;ArmServo_PollDemo(1,1);angles(0);
 command(0x21,50,50,3,0);command(0x24,0,0,0,0);angles(5);
 fake_primask=1;press(1);assert(fake_primask==1);angles(0);fake_primask=0;
 prepare();fail_start=1;assert(ArmServo_Init()==HAL_ERROR);assert(started==0 && stopped==1);
 press(0);angles(0);
 prepare();fail_config=2;assert(ArmServo_Init()==HAL_ERROR);assert(started==0);
 prepare();clock_hz=72000000;fake_rcc.CFGR=0;assert(ArmServo_Init()==HAL_OK);
 assert(saved_timer.Init.Prescaler==71);
 prepare();clock_hz=36000001;assert(ArmServo_Init()==HAL_ERROR);assert(started==0);
 assert(ArmServo_GetTargetAngle(3)==0 && ArmServo_GetTargetAngle(4)==0);
 puts("PASS arm servo: 4 configured/2 started, reset zero, +/-5, hold/no-repeat, debounce, limits, both-key block, held reset, tick wrap, PWM failure cleanup, timer clocks.");return 0;
}
