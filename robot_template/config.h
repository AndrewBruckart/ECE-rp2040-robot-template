#pragma once

#define PIN_BTN_START 16
#define PIN_BTN_STOP  17

#define PIN_LEFT_A 24
#define PIN_LEFT_B 23
#define PIN_RIGHT_A 22
#define PIN_RIGHT_B 21

#define PIN_LED_RED 10
#define PIN_LED_GREEN 11
#define PIN_LED_YELLOW 12
#define PIN_SPEAKER 13
#define PIN_STEER_SERVO 14

#define PIN_ENC_B 20
#define PIN_ENC_SW 19
#define PIN_ENC_A 18

#define PIN_BRAKE_LIGHT 25
#define PIN_LEFT_TURN 2
#define PIN_RIGHT_TURN 3

#define PIN_CDS A0
#define PIN_IR_RIGHT A1
#define PIN_IR_CENTER A2
#define PIN_IR_LEFT A3

#define OLED_SDA 4
#define OLED_SCL 5
#define OLED_ADDR 0x3C

#define ACCEL_ADDR 0x1C
#define ADC_RESOLUTION_BITS 12

#define SERVO_CENTER_US 1500
#define SERVO_CENTER_TRIM_US 30
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000

// Wall-follow tuning now lives in wall_follow.cpp so the controller can use
// calibrated inches instead of hard-coded raw ADC thresholds.
