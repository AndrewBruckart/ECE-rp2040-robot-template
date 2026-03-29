#pragma once

#include <Arduino.h>

void initMotors();
void setMotorRawLeft(int8_t cmd);
void setMotorRawRight(int8_t cmd);
void setMotorSpeedPercent(int percent);
int getMotorSpeedPercent();
void motorsBrake();
void motorsCoast();
