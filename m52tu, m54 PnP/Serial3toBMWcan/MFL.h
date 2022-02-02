// For reading cruise buttons in e46, e39 etc. BMW cars using single wire to transmit MFL data.
// Original implementation created by Robin and modified by pazi88

#ifndef MFL_H
#define MFL_H

volatile unsigned long cruiseBuffer[8];
volatile byte bufferC;
volatile unsigned long prev_time;

volatile bool update_cruise;
volatile bool startByte;
bool MFL_CRUISE_MINUS;
bool MFL_CRUISE_PLUS;
bool MFL_CRUISE_IO;
bool MFL_CRUISE_ON;

byte ioC;
byte onC;
byte plusC;
byte minusC;

#define CRUISE_CONTROL_PIN PB13  //Input pin for MFL
#define CRUISE_LOW_TIME_THRES 5000  //There is 10ms between messages. If there has been more than 5ms from last falling edge, we can for sure say that new message has started.
#define CRUISE_HIGH_TIME_THRES 250  //Bit 0 is roughly 168us long and Bit 1 is roughly 336us long. So we set this thresold to the middle point of that.
#define CRUISE_CONTROL_BUTTON_COUNT 2 //How many messages we need to see before we define if button has been pressed or released.

#define BUTTON_PRESSED      1
#define BUTTON_UNPRESSED    0

void setupMFL();
void updateCruise();
void readCruiseHigh();
void readCruiseLow();

#endif