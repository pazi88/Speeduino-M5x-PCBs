// Totally untested code for reading cruise buttons. Do not use for anything.

volatile unsigned long cruiseBuffer[8];
volatile byte bufferC = 0;
volatile unsigned long prev_time = 0;

volatile bool update_cruise = false;
volatile bool startByte = false;
bool MFL_CRUISE_MINUS;
bool MFL_CRUISE_PLUS;
bool MFL_CRUISE_IO;
bool MFL_CRUISE_ON;

byte ioC = 0;
byte onC = 0;
byte plusC = 0;
byte minusC = 0;

#define CRUISE_CONTROL_PIN PB13  // Input pin for MFL
#define CRUISE_LOW_TIME_THRES 5000  // There is 10ms between messages. If there has been more than 5ms from last falling edge, we can for sure say that new message has started.
#define CRUISE_HIGH_TIME_THRES 400
#define CRUISE_CONTROL_BUTTON_COUNT 4 // How many messages we need to see before we define if button has been pressed or released.

#define BUTTON_PRESSED      1
#define BUTTON_UNPRESSED    0


void setup() {
  Serial.begin(115200); // for debugging
  pinMode(CRUISE_CONTROL_PIN, INPUT);
  attachInterrupt(CRUISE_CONTROL_PIN, &readCruiseLow, FALLING);

}

void updateCruise() {

  if (update_cruise) {
    unsigned long cruiseBufferCopy[8];

    for (byte i = 0; i < 8; i++) {
      cruiseBufferCopy[i] = cruiseBuffer[i];
	  Serial.print(cruiseBuffer[i]);
	  Serial.print("; ");
      cruiseBuffer[i] = 0;
    }
	Serial.println();

    update_cruise = false;

    //Reconstruct byte
    int value = 0;
    //last one seems to be a counter
    for (byte i = 0; i < 7; i++) {
      if (cruiseBufferCopy[i] > CRUISE_HIGH_TIME_THRES) value |= 1 << (7 - i);
    }
  
    if (value == 252 && minusC < CRUISE_CONTROL_BUTTON_COUNT) {
      minusC++;

      if (minusC == CRUISE_CONTROL_BUTTON_COUNT) MFL_CRUISE_MINUS = BUTTON_PRESSED;
      Serial.println(F("Speed Minus"));
      return;
    }
    else if (minusC > 0) {
      minusC--;

      if (minusC == 0) MFL_CRUISE_MINUS = BUTTON_UNPRESSED;
    }

    if (value == 182 && plusC < CRUISE_CONTROL_BUTTON_COUNT) {
      plusC++;
      if (plusC == CRUISE_CONTROL_BUTTON_COUNT) MFL_CRUISE_PLUS = BUTTON_PRESSED;
      return;
        Serial.println(F("Speed Plus"));
    }
    else if (plusC > 0) {
      plusC--;

      if (plusC == 0) MFL_CRUISE_PLUS = BUTTON_UNPRESSED;
    }

    if (value == 218 && ioC < CRUISE_CONTROL_BUTTON_COUNT) {
      ioC++;

      if (ioC == CRUISE_CONTROL_BUTTON_COUNT) MFL_CRUISE_IO = BUTTON_PRESSED;
      return;
      Serial.println(F("IO"));
    }
    else if (ioC > 0) {
      ioC--;

      if (ioC == 0) MFL_CRUISE_IO = BUTTON_UNPRESSED;
    }

    if (value == 110 && onC < CRUISE_CONTROL_BUTTON_COUNT) {
      onC++;
      if (onC == CRUISE_CONTROL_BUTTON_COUNT) MFL_CRUISE_ON = BUTTON_PRESSED;
      return;
        Serial.println(F("Cruise On"));
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

void loop() {
  // put your main code here, to run repeatedly:

}
