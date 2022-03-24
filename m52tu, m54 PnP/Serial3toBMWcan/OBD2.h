// For communicating OBD2 data over K-line


#ifndef OBD2_H
#define OBD2_H

#define CALIBRATION_TEMPERATURE_OFFSET 40

uint8_t calculateModulo256(uint8_t byteNumber);
void sendOBDinit();
void obd_response(uint8_t therequestedPID , uint8_t therequestedPIDlow, uint8_t therequestedPIDhigh);

#endif