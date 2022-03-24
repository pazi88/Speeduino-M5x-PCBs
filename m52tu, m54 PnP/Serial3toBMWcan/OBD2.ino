#include "OBD2.h"

void sendOBDinit()
{
  Serial2.write(0x83);
  Serial2.write(0xF1);
  Serial2.write(0x11);
  Serial2.write(0xC1);
  Serial2.write(0x8F);
  Serial2.write(0xEF);
  Serial2.write(0xC4);
  Serial2.flush();
  Serial2.read();
  Serial2.read();
  Serial2.read();
  Serial2.read();
  Serial2.read();
  Serial2.read();
  Serial2.read();
  while (1){
    if( Serial2.available() > 0){ Serial.print(Serial2.read()); }
  }
}

uint8_t calculateModulo256(uint8_t byteNumber)
{
  uint8_t modulo256 = 0;    //modulo256 checksum
  for (int i = 0; i <= byteNumber; i++) {
    modulo256 += data[i];
  }
  return modulo256;
}

void obd_response(uint8_t PIDmode, uint8_t requestedPIDlow, uint8_t requestedPIDhigh)
{
//only build the PID if the mcu has onboard/attached can 

  uint16_t obdcalcA;    //used in obd calcs
  uint16_t obdcalcB;    //used in obd calcs 
  uint16_t obdcalcC;    //used in obd calcs 
  uint16_t obdcalcD;    //used in obd calcs
  uint32_t obdcalcE32;    //used in calcs 
  uint32_t obdcalcF32;    //used in calcs 
  uint16_t obdcalcG16;    //used in calcs
  uint16_t obdcalcH16;    //used in calcs
  
  if (PIDmode == 0x01)
  {
    //currentStatus.canin[13] = therequestedPIDlow; 
    switch (requestedPIDlow)
    {
      case 0:       //PID-0x00 PIDs supported 01-20  
        data[0] = 0x86;    // sending 4 bytes (82h + number of bytes)
        data[1] = 0xF1;
        data[2] = 0x11;
        data[3] = 0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
        data[4] = 0x00;    // PID code
        data[5] = 0x08;   //B0000 1000   1-8
        data[6] = B01111110;   //9-16
        data[7] = B10100000;   //17-24
        data[8] = B00010001;   //17-32
        data[9] = calculateModulo256(9);
      break;
   
      case 5:      //PID-0x05 Engine coolant temperature , range is -40 to 215 deg C , formula == A-40
        data[0] = 0x83;    // sending 1 byte (82h + number of bytes)
        data[1] = 0xF1;
        data[2] = 0x11;
        data[3] = 0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
        data[4] = 0x05;    // PID code
        data[5] = (byte)(currentStatus.CLT + CALIBRATION_TEMPERATURE_OFFSET);   //the data value A
        data[6] = calculateModulo256(6);
      break;
   
      case 10:        // PID-0x0A , Fuel Pressure (Gauge) , range is 0 to 765 kPa , formula == A / 3)
        data[0] = 0x83;    // sending 1 byte (82h + number of bytes)
        data[1] = 0xF1;
        data[2] = 0x11;
        data[3] = 0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
        data[4] = 0x0A;    // PID code
        data[5] = 20;
        data[6] = calculateModulo256(6);
      break;
   
      case 11:        // PID-0x0B , MAP , range is 0 to 255 kPa , Formula == A
        data[0] = 0x83;    // sending 1 byte (82h + number of bytes)
        data[1] = 0xF1;
        data[2] = 0x11;
        data[3] = 0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
        data[4] = 0x0B;    // PID code
        data[4] = lowByte(currentStatus.MAP);    // absolute map
        data[6] = calculateModulo256(6);
      break;
   
      case 12:        // PID-0x0C , RPM  , range is 0 to 16383.75 rpm , Formula == 256A+B / 4
        uint16_t temp_revs; 
        temp_revs = currentStatus.RPM << 2 ;      //
        data[0] = 0x84;    // sending 2 bytes (82h + number of bytes)
        data[1] = 0xF1;
        data[2] = 0x11;
        data[3] = 0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
        data[4] = 0x00;    // PID code
        data[5] = highByte(temp_revs);         //obdcalcB; A
        data[6] = lowByte(temp_revs);          //obdcalcD; B
        data[7] = calculateModulo256(7);
      break;
      default:
      break;
    }
  }
}