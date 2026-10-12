#ifndef ARM_SERVO_HAL_STUB_H
#define ARM_SERVO_HAL_STUB_H
#define __MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum {HAL_OK,HAL_ERROR} HAL_StatusTypeDef;
typedef struct {uint32_t CFGR;} RCC_TypeDef;
extern RCC_TypeDef fake_rcc;
#define RCC (&fake_rcc)
#define RCC_CFGR_PPRE2 0xE000U
#define TIM8 ((void *)8)
#define GPIOC ((void *)3)
#define GPIOE ((void *)5)
#define GPIO_PIN_3 (1U<<3)
#define GPIO_PIN_4 (1U<<4)
#define GPIO_PIN_6 (1U<<6)
#define GPIO_PIN_7 (1U<<7)
#define GPIO_PIN_8 (1U<<8)
#define GPIO_PIN_9 (1U<<9)
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
#define GPIO_MODE_AF_PP 2U
#define GPIO_PULLDOWN 2U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_AF3_TIM8 3U
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define TIM_COUNTERMODE_UP 0U
#define TIM_CLOCKDIVISION_DIV1 0U
#define TIM_AUTORELOAD_PRELOAD_ENABLE 1U
#define TIM_OCMODE_PWM1 6U
#define TIM_OCPOLARITY_HIGH 0U
#define TIM_OCNPOLARITY_HIGH 0U
#define TIM_OCFAST_DISABLE 0U
#define TIM_OCIDLESTATE_RESET 0U
#define TIM_OCNIDLESTATE_RESET 0U
#define TIM_EVENTSOURCE_UPDATE 1U
typedef struct {uint32_t Prescaler,CounterMode,Period,ClockDivision,RepetitionCounter,AutoReloadPreload;} TIM_Base_InitTypeDef;
typedef struct {void *Instance;TIM_Base_InitTypeDef Init;} TIM_HandleTypeDef;
typedef struct {uint32_t Pin,Mode,Pull,Speed,Alternate;} GPIO_InitTypeDef;
typedef struct {uint32_t OCMode,Pulse,OCPolarity,OCNPolarity,OCFastMode,OCIdleState,OCNIdleState;} TIM_OC_InitTypeDef;
extern uint32_t fake_primask;
static inline uint32_t __get_PRIMASK(void){return fake_primask;}
static inline void __disable_irq(void){fake_primask=1;}
static inline void __enable_irq(void){fake_primask=0;}
extern uint32_t fake_compare[4];
extern unsigned fake_clock_flags;
#define __HAL_RCC_TIM8_CLK_ENABLE() (fake_clock_flags|=1U)
#define __HAL_RCC_GPIOC_CLK_ENABLE() (fake_clock_flags|=2U)
#define __HAL_TIM_SET_COMPARE(t,ch,value) ((void)(t),fake_compare[(ch)/4]=(value))
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *timer,uint32_t event);
uint32_t HAL_RCC_GetPCLK2Freq(void);
uint32_t HAL_GetTick(void);
uint8_t HAL_GPIO_ReadPin(void *port,uint32_t pin);
void HAL_GPIO_Init(void *port,GPIO_InitTypeDef *gpio);
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *timer,TIM_OC_InitTypeDef *oc,uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer,uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer,uint32_t channel);
#endif
