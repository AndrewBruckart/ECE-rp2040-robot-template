#include "sensors.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

static Adafruit_MMA8451 mma = Adafruit_MMA8451();
static bool accelReady = false;

static int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

static int readMedianAnalog(int pin) {
  int a = analogRead(pin);
  int b = analogRead(pin);
  int c = analogRead(pin);
  return median3(a, b, c);
}

void initSensors() {
  analogReadResolution(ADC_RESOLUTION_BITS);
  Wire.begin();
  accelReady = mma.begin(ACCEL_ADDR, &Wire);
  if (accelReady) {
    mma.setRange(MMA8451_RANGE_4_G);
    mma.setDataRate(MMA8451_DATARATE_100_HZ);
  }
}

int readIRLeft() { return readMedianAnalog(PIN_IR_LEFT); }
int readIRCenter() { return readMedianAnalog(PIN_IR_CENTER); }
int readIRRight() { return readMedianAnalog(PIN_IR_RIGHT); }
int readLDR() { return readMedianAnalog(PIN_CDS); }

bool accelAvailable() {
  return accelReady;
}

void readAccelXYZ(int &x, int &y, int &z) {
  x = 0;
  y = 0;
  z = 0;
  if (!accelReady) {
    return;
  }
  mma.read();
  x = (int)(mma.x_g * 1000.0f);
  y = (int)(mma.y_g * 1000.0f);
  z = (int)(mma.z_g * 1000.0f);
}

void setAccelDataRate(mma8451_dataRate_t dataRate) {
  if (!accelReady) {
    return;
  }
  mma.setDataRate(dataRate);
}
