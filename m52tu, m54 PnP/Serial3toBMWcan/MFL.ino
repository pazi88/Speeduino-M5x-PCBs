// For reading cruise buttons in e46, e39 etc. BMW cars using single wire to transmit MFL data.
// Original implementation created by Robin and modified by pazi88

#include "MFL.h"

void setupMFL() {
  pinMode(CRUISE_CONTROL_PIN, INPUT);
  cruiseBuffer[8];
  byte bufferC = 0;
  prev_time = 0;
  update_cruise = false;
  startByte = false;
  ioC = 0;
  onC = 0;
  plusC = 0;
  minusC = 0;
  attachInterrupt(CRUISE_CONTROL_PIN, &readCruiseLow, FALLING);
}

void updateCruise() {

  if (update_cruise) {
    unsigned long cruiseBufferCopy[8];

    for (byte i = 0; i < 8; i++) {
      cruiseBufferCopy[i] = cruiseBuffer[i];
      cruiseBuffer[i] = 0;
    }

    update_cruise = false;

    //Reconstruct byte
    int value = 0;
    //last one seems to be a counter
    for (byte i = 0; i < 7; i++) {
      if (cruiseBufferCopy[i] > CRUISE_HIGH_TIME_THRES) value |= 1 << (7 - i);
    }
  
    if (value == 254 && minusC < CRUISE_CONTROL_BUTTON_COUNT) {
      minusC++;

      if (minusC == CRUISE_CONTROL_BUTTON_COUNT) MFL_CRUISE_MINUS = BUTTON_PRESSED;
      Serial.println(F("Speed Minus"));
      return;
    }
    else if (minusC > 0) {
      minusC--;

      if (minusC == 0) MFL_CRUISE_MINUS = BUTTON_UNPRESSED;
    }

    if (value == 146 && plusC < CRUISE_CONTROL_BUTTON_COUNT) {
      plusC++;
      if (plusC == CRUISE_CONTROL_BUTTON_COUNT) MFL_CRUISE_PLUS = BUTTON_PRESSED;
      Serial.println(F("Speed Plus"));
      return;

    }
    else if (plusC > 0) {
      plusC--;

      if (plusC == 0) MFL_CRUISE_PLUS = BUTTON_UNPRESSED;
    }

    if (value == 216 && ioC < CRUISE_CONTROL_BUTTON_COUNT) {
      ioC++;

      if (ioC == CRUISE_CONTROL_BUTTON_COUNT) MFL_CRUISE_IO = BUTTON_PRESSED;
      Serial.println(F("IO"));
      return;
    }
    else if (ioC > 0) {
      ioC--;

      if (ioC == 0) MFL_CRUISE_IO = BUTTON_UNPRESSED;
    }

    if (value == 74 && onC < CRUISE_CONTROL_BUTTON_COUNT) {
      onC++;
      if (onC == CRUISE_CONTROL_BUTTON_COUNT) MFL_CRUISE_ON = BUTTON_PRESSED;
        Serial.println(F("Cruise On"));
      return;
    }
    else if (onC > 0) {
      onC--;

      if (onC == 0) MFL_CRUISE_ON = BUTTON_UNPRESSED;
    }
  }
}

//Rising
void readCruiseHigh() {
  attachInterrupt(CRUISE_CONTROL_PIN, &readCruiseLow, FALLING);

  prev_time = micros();
}

//Falling
void readCruiseLow() {
  attachInterrupt(CRUISE_CONTROL_PIN, &readCruiseHigh, RISING);
  unsigned long pwm_value = micros() - prev_time;
  prev_time = micros();
  //Start of new PWM Signal
  if (pwm_value > CRUISE_LOW_TIME_THRES && !startByte) {
    bufferC = 0;
    startByte = true;
  }
  else {
    //Check if length of last buffer is correct (what we recorded so far)
    if(bufferC < 8 && startByte)
    {
      cruiseBuffer[bufferC] = pwm_value;
      bufferC = bufferC + 1;
    }

    if (bufferC >= 8 && !update_cruise) {
      update_cruise = true;
      startByte = false;
    }
  }
}