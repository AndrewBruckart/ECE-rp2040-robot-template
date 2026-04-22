#pragma once

#include <Adafruit_MMA8451.h>

enum IrSensorId {
  IR_SENSOR_LEFT = 0,
  IR_SENSOR_CENTER,
  IR_SENSOR_RIGHT
};

void initSensors();
int readIRLeft();
int readIRCenter();
int readIRRight();
float readIRDistanceInches(IrSensorId sensor);
float convertIRRawToDistanceInches(IrSensorId sensor, int rawAdc);
int readLDR();
bool accelAvailable();
void readAccelXYZ(int &x, int &y, int &z);
void setAccelDataRate(mma8451_dataRate_t dataRate);
