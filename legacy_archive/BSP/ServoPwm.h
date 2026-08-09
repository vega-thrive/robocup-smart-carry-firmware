#ifndef __SERVO_PWM_H__
#define __SERVO_PWM_H__

#include <stdint.h>

#define SERVO_PWM_MIN_US     500U
#define SERVO_PWM_CENTER_US 1500U
#define SERVO_PWM_MAX_US    2500U

void ServoPwm_Init(void);
void ServoPwm_SetPulse(uint8_t channel, uint16_t pulse_us);
void ServoPwm_SetBoth(uint16_t servo1_us, uint16_t servo2_us);

#endif
