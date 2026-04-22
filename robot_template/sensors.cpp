#include "sensors.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

struct IrCalibrationPoint {
  float distanceInches;
  int adc;
};

static Adafruit_MMA8451 mma = Adafruit_MMA8451();
static bool accelReady = false;
static const float IR_NO_WALL_DISTANCE_INCHES = 18.0f;

// These points come from Sensor Calibration.xlsx and represent distance from
// the car body to the wall for the left/right sensors and front bumper to wall
// for the center sensor.
static const IrCalibrationPoint IR_CAL_LEFT[] = {
  {0.0f, 205}, {1.0f, 1533}, {2.0f, 2397}, {3.0f, 3025},
  {4.0f, 3353}, {5.0f, 3409}, {6.0f, 3537}, {7.0f, 3580},
  {8.0f, 3600}, {9.0f, 3640}, {10.0f, 3660}, {IR_NO_WALL_DISTANCE_INCHES, 3817}
};

static const IrCalibrationPoint IR_CAL_CENTER[] = {
  {0.0f, 128}, {1.0f, 243}, {2.0f, 273}, {3.0f, 349},
  {4.0f, 890}, {5.0f, 1457}, {6.0f, 1900}, {7.0f, 2265},
  {8.0f, 2513}, {9.0f, 2745}, {10.0f, 2890}, {11.0f, 3000},
  {12.0f, 3125}, {13.0f, 3200}, {14.0f, 3265}, {IR_NO_WALL_DISTANCE_INCHES, 3717}
};

static const IrCalibrationPoint IR_CAL_RIGHT[] = {
  {0.0f, 190}, {1.0f, 555}, {2.0f, 2197}, {3.0f, 2937},
  {4.0f, 3269}, {5.0f, 3473}, {6.0f, 3545}, {7.0f, 3609},
  {8.0f, 3656}, {9.0f, 3700}, {10.0f, 3720}, {IR_NO_WALL_DISTANCE_INCHES, 3792}
};

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

static const IrCalibrationPoint *getCalibrationTable(IrSensorId sensor, int &count) {
  switch (sensor) {
    case IR_SENSOR_LEFT:
      count = sizeof(IR_CAL_LEFT) / sizeof(IR_CAL_LEFT[0]);
      return IR_CAL_LEFT;
    case IR_SENSOR_RIGHT:
      count = sizeof(IR_CAL_RIGHT) / sizeof(IR_CAL_RIGHT[0]);
      return IR_CAL_RIGHT;
    case IR_SENSOR_CENTER:
    default:
      count = sizeof(IR_CAL_CENTER) / sizeof(IR_CAL_CENTER[0]);
      return IR_CAL_CENTER;
  }
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

float convertIRRawToDistanceInches(IrSensorId sensor, int rawAdc) {
  int pointCount = 0;
  const IrCalibrationPoint *table = getCalibrationTable(sensor, pointCount);
  if (pointCount <= 0) {
    return 0.0f;
  }

  if (rawAdc <= table[0].adc) {
    return table[0].distanceInches;
  }

  for (int i = 1; i < pointCount; ++i) {
    if (rawAdc <= table[i].adc) {
      const IrCalibrationPoint &a = table[i - 1];
      const IrCalibrationPoint &b = table[i];
      int adcSpan = b.adc - a.adc;
      if (adcSpan <= 0) {
        return b.distanceInches;
      }
      float ratio = (float)(rawAdc - a.adc) / (float)adcSpan;
      return a.distanceInches + ratio * (b.distanceInches - a.distanceInches);
    }
  }

  return table[pointCount - 1].distanceInches;
}

float readIRDistanceInches(IrSensorId sensor) {
  switch (sensor) {
    case IR_SENSOR_LEFT:
      return convertIRRawToDistanceInches(sensor, readIRLeft());
    case IR_SENSOR_RIGHT:
      return convertIRRawToDistanceInches(sensor, readIRRight());
    case IR_SENSOR_CENTER:
    default:
      return convertIRRawToDistanceInches(sensor, readIRCenter());
  }
}

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
