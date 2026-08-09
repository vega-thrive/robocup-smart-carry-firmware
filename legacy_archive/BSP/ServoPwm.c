#include "ServoPwm.h"
#include "main.h"

static uint16_t ServoPwm_ClampPulse(uint16_t pulse_us)
{
    if (pulse_us < SERVO_PWM_MIN_US) return SERVO_PWM_MIN_US;
    if (pulse_us > SERVO_PWM_MAX_US) return SERVO_PWM_MAX_US;
    return pulse_us;
}

void ServoPwm_Init(void)
{
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_TIM13_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();

    /* PF8 = TIM13_CH1, PF9 = TIM14_CH1, both AF9. */
    GPIOF->MODER = (GPIOF->MODER & ~((3UL << 16) | (3UL << 18)))
                 |  ((2UL << 16) | (2UL << 18));
    GPIOF->OTYPER &= ~(GPIO_PIN_8 | GPIO_PIN_9);
    GPIOF->PUPDR &= ~((3UL << 16) | (3UL << 18));
    GPIOF->OSPEEDR |= (3UL << 16) | (3UL << 18);
    GPIOF->AFR[1] = (GPIOF->AFR[1] & ~((0xFUL << 0) | (0xFUL << 4)))
                  |  (9UL << 0) | (9UL << 4);

    /* APB1 timer clock 84 MHz: 84 MHz / 84 = 1 MHz; 20000 us = 50 Hz. */
    TIM13->CR1 = 0;
    TIM13->PSC = 83U;
    TIM13->ARR = 19999U;
    TIM13->CCR1 = SERVO_PWM_CENTER_US;
    TIM13->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
    TIM13->CCER = TIM_CCER_CC1E;
    TIM13->EGR = TIM_EGR_UG;
    TIM13->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    TIM14->CR1 = 0;
    TIM14->PSC = 83U;
    TIM14->ARR = 19999U;
    TIM14->CCR1 = SERVO_PWM_CENTER_US;
    TIM14->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
    TIM14->CCER = TIM_CCER_CC1E;
    TIM14->EGR = TIM_EGR_UG;
    TIM14->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void ServoPwm_SetPulse(uint8_t channel, uint16_t pulse_us)
{
    pulse_us = ServoPwm_ClampPulse(pulse_us);
    if (channel == 1U)
    {
        TIM13->CCR1 = pulse_us;
    }
    else if (channel == 2U)
    {
        TIM14->CCR1 = pulse_us;
    }
}

void ServoPwm_SetBoth(uint16_t servo1_us, uint16_t servo2_us)
{
    ServoPwm_SetPulse(1U, servo1_us);
    ServoPwm_SetPulse(2U, servo2_us);
}
