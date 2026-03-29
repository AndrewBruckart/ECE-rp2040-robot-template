#pragma once

#include <Adafruit_MMA8451.h>

void initSensors();
int readIRLeft();
int readIRCenter();
int readIRRight();
int readLDR();
bool accelAvailable();
void readAccelXYZ(int &x, int &y, int &z);
void setAccelDataRate(mma8451_dataRate_t dataRate);
