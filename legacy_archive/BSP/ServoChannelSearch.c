#include "ServoChannelSearch.h"
#include "main.h"
#include "LCD.h"
#include "Task1Mechanism.h"
#include <stdio.h>

#define SERVO_CHANNEL_COUNT   8U
#define SERVO_FRAME_US    20000U
#define SERVO_CENTER_US    1500U
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} ServoSearchPin;

static const ServoSearchPin servo_pins[SERVO_CHANNEL_COUNT] =
{
    {GPIOC, GPIO_PIN_2}, /* Gray2 channel 1 */
    {GPIOC, GPIO_PIN_3}, /* Gray2 channel 2 */
    {GPIOA, GPIO_PIN_4}, /* Gray2 channel 3 */
    {GPIOA, GPIO_PIN_5}, /* Gray2 channel 4 */
    {GPIOA, GPIO_PIN_6}, /* Gray2 channel 5 */
    {GPIOA, GPIO_PIN_7}, /* Gray2 channel 6 */
    {GPIOC, GPIO_PIN_4}, /* Gray2 channel 7 */
    {GPIOC, GPIO_PIN_5}  /* Gray2 channel 8 */
};

static void ServoSearch_DelayUs(uint32_t delay_us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = delay_us * (SystemCoreClock / 1000000U);
    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
    }
}

static void ServoSearch_AllLow(void)
{
    uint8_t channel;
    for (channel = 0U; channel < SERVO_CHANNEL_COUNT; channel++)
    {
        HAL_GPIO_WritePin(servo_pins[channel].port,
                          servo_pins[channel].pin, GPIO_PIN_RESET);
    }
}

static void ServoSearch_InitPins(void)
{
    GPIO_InitTypeDef init = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    ServoSearch_AllLow();

    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    init.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &init);
    init.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &init);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void ServoSearch_HoldFirstPair(uint16_t channel1_us,
                                      uint16_t channel2_us,
                                      uint32_t duration_ms)
{
    uint32_t frames = duration_ms / 20U;
    uint32_t frame;
    uint16_t first_edge;
    uint16_t second_edge;
    const ServoSearchPin *a = &servo_pins[0];
    const ServoSearchPin *b = &servo_pins[1];

    for (frame = 0U; frame < frames; frame++)
    {
        if (channel1_us != 0U)
        {
            HAL_GPIO_WritePin(a->port, a->pin, GPIO_PIN_SET);
        }
        if (channel2_us != 0U)
        {
            HAL_GPIO_WritePin(b->port, b->pin, GPIO_PIN_SET);
        }

        first_edge = (channel1_us < channel2_us) ? channel1_us : channel2_us;
        second_edge = (channel1_us < channel2_us) ? channel2_us : channel1_us;
        ServoSearch_DelayUs(first_edge);
        if ((channel1_us != 0U) && (channel1_us <= channel2_us))
        {
            HAL_GPIO_WritePin(a->port, a->pin, GPIO_PIN_RESET);
        }
        if ((channel2_us != 0U) && (channel2_us <= channel1_us))
        {
            HAL_GPIO_WritePin(b->port, b->pin, GPIO_PIN_RESET);
        }
        ServoSearch_DelayUs(second_edge - first_edge);
        HAL_GPIO_WritePin(a->port, a->pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(b->port, b->pin, GPIO_PIN_RESET);
        ServoSearch_DelayUs(SERVO_FRAME_US - second_edge);
    }
}

static void ServoSearch_Show(const Task1ArmPose *pose)
{
    char line[32];
    snprintf(line, sizeof(line), "CH1 BIG: %u us", pose->big_arm_us);
    LCD_DrawString_NoDMA(1, 80, "                    ",
                         LCD_WHITE, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 80, line, LCD_GREEN, LCD_BLACK);
    if (pose->small_arm_us == 0U)
    {
        snprintf(line, sizeof(line), "CH2 SMALL: PWM OFF");
    }
    else
    {
        snprintf(line, sizeof(line), "CH2 SMALL: %u us", pose->small_arm_us);
    }
    LCD_DrawString_NoDMA(1, 120, "                    ",
                         LCD_WHITE, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 120, line, LCD_WHITE, LCD_BLACK);
}

void ServoChannelSearch_Run(void)
{
    const Task1ArmPose *pose = Task1Mechanism_GetWorkingPose();

    ServoSearch_InitPins();
    LCD_FillScreen(LCD_BLACK);
    LCD_DrawString_NoDMA(1, 20, "ARM HOLD TEST",
                         LCD_WHITE, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 180, "TASK1 WORKING POSE",
                         LCD_WHITE, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 220, "MOTORS/QR DISABLED",
                         LCD_RED, LCD_BLACK);

    while (1)
    {
        ServoSearch_Show(pose);
        ServoSearch_HoldFirstPair(pose->big_arm_us,
                                  pose->small_arm_us, 1000U);
    }
}
