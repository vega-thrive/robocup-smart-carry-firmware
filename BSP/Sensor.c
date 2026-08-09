  #include "Sensor.h"

/* 前八路灰度引脚表 (索引 1~8, 对应 SENSOR01~08) */
static const struct { GPIO_TypeDef *port; uint16_t pin; } front_pins[9] = {
    {NULL, 0},
    {SENSOR01_GPIO_Port, SENSOR01_Pin},
    {SENSOR02_GPIO_Port, SENSOR02_Pin},
    {SENSOR03_GPIO_Port, SENSOR03_Pin},
    {SENSOR04_GPIO_Port, SENSOR04_Pin},
    {SENSOR05_GPIO_Port, SENSOR05_Pin},
    {SENSOR06_GPIO_Port, SENSOR06_Pin},
    {SENSOR07_GPIO_Port, SENSOR07_Pin},
    {SENSOR08_GPIO_Port, SENSOR08_Pin},
};

/* 后八路灰度引脚表 (索引 1~8, 对应 SENSOR17~24) */
static const struct { GPIO_TypeDef *port; uint16_t pin; } rear_pins[9] = {
    {NULL, 0},
    {SENSOR17_GPIO_Port, SENSOR17_Pin},
    {SENSOR18_GPIO_Port, SENSOR18_Pin},
    {SENSOR19_GPIO_Port, SENSOR19_Pin},
    {SENSOR20_GPIO_Port, SENSOR20_Pin},
    {SENSOR21_GPIO_Port, SENSOR21_Pin},
    {SENSOR22_GPIO_Port, SENSOR22_Pin},
    {SENSOR23_GPIO_Port, SENSOR23_Pin},
    {SENSOR24_GPIO_Port, SENSOR24_Pin},
};

uint8_t GS_state[9];
uint8_t Rear_GS_state[9];

void Sensor_Get(void)
{
    uint8_t i;
    for (i = 1U; i <= 8U; i++)
        GS_state[i] = (HAL_GPIO_ReadPin(front_pins[i].port, front_pins[i].pin)
                       == GPIO_PIN_SET) ? 1U : 0U;
}

void RearSensor_Get(void)
{
    uint8_t i;
    for (i = 1U; i <= 8U; i++)
        Rear_GS_state[i] = (HAL_GPIO_ReadPin(rear_pins[i].port, rear_pins[i].pin)
                            == GPIO_PIN_SET) ? 1U : 0U;
}
