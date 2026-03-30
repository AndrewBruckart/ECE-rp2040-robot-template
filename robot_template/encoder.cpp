#include "encoder.h"
#include "config.h"

volatile int32_t enc_pending = 0;

static void encoderISR() {
  int a = digitalRead(PIN_ENC_A);
  int b = digitalRead(PIN_ENC_B);
  if (a == b) {
    enc_pending++;
  } else {
    enc_pending--;
  }
}

void initEncoder() {
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  pinMode(PIN_ENC_SW, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
}

int consumeEncoderClicks() {
  noInterrupts();
  int32_t value = enc_pending;
  enc_pending = 0;
  interrupts();
  return (int)value;
}
