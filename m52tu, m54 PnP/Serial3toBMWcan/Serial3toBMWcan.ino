// This code is meant to read real time data from Speeduino EFI using serial3 connection in speeduino and convert that to CAN messages for BMW e39/e46 instrument clusters
// The hardware that the code is meant to be used is STM32F103C8T6 STM32 Development Board (BluePill) with MCP2551 transceiver.
// Created by pazi88 and there is no guarantee at all that any of this will work. Tested with https://github.com/stm32duino/Arduino_Core_STM32 board manager in Arduino IDE

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
#include <src/STM32_CAN/STM32_CAN.h>
#define DS2_ENABLE
#ifdef DS2_ENABLE
  #include "DS2.h"
#endif

#ifdef ARDUINO_BLUEPILL_F103C8
  HardwareSerial Serial3(USART3); //for some reason this isn't defined in arduino_core_stm32
  #ifdef DS2_ENABLE
    HardwareSerial Serial2(USART2); //for some reason this isn't defined in arduino_core_stm32
    DS2 DS2(Serial2);
  #endif
#endif
#define pin  LED_BUILTIN

static CAN_message_t CAN_msg_RPM;
static CAN_message_t CAN_msg_CLT_TPS;
static CAN_message_t CAN_msg_MPG_CEL;
static CAN_message_t CAN_inMsg;

STM32_CAN Can1 (_CAN1,DEF);

// This struct gathers data read from speeduino
struct statuses {
	uint8_t secl; //secl is simply a counter that increments each second.
	uint8_t status1; //status1 Bitfield, inj1Status(0), inj2Status(1), inj3Status(2), inj4Status(3), DFCOOn(4), boostCutFuel(5), toothLog1Ready(6), toothLog2Ready(7)
	uint8_t engine; //Engine Status Bitfield, running(0), crank(1), ase(2), warmup(3), tpsaccaen(4), tpsacden(5), mapaccaen(6), mapaccden(7)
	uint8_t dwell; //Dwell in ms * 10
	uint16_t MAP; //2 bytes for MAP
	uint8_t IAT;
	uint8_t CLT;
	uint8_t batCorrection; //Battery voltage correction (%)
	uint8_t battery10; //battery voltage
	uint8_t O2; //O2
	uint8_t egoCorrection; //Exhaust gas correction (%)
	uint8_t iatCorrection; //Air temperature Correction (%)
	uint8_t wueCorrection; //Warmup enrichment (%)
	uint16_t RPM; //rpm
	uint8_t AEamount; //acceleration enrichment (%)
	uint8_t corrections; //Total GammaE (%)
	uint8_t VE; //Current VE 1 (%)
	uint8_t afrTarget;
	uint16_t PW1; //Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
	uint8_t tpsDOT; //TPS DOT
	int8_t advance;
	uint8_t TPS; // TPS (0% to 100%)
	uint16_t loopsPerSecond;
	uint16_t freeRAM;
	uint8_t boostTarget; //boost target divided by 2 to fit in a byte
	uint8_t boostDuty;
	uint8_t spark; //Spark related bitfield, launchHard(0), launchSoft(1), hardLimitOn(2), softLimitOn(3), boostCutSpark(4), error(5), idleControlOn(6), sync(7)
	uint16_t rpmDOT;
	uint8_t ethanolPct; //Flex sensor value (or 0 if not used)
	uint8_t flexCorrection; //Flex fuel correction (% above or below 100)
	uint8_t flexIgnCorrection; //Ignition correction (Increased degrees of advance) for flex fuel
	uint8_t idleLoad;
	uint8_t testOutputs; // testEnabled(0), testActive(1)
	uint8_t O2_2; //O2
	uint8_t baro; //Barometer value
	uint16_t CANin_1;
	uint16_t CANin_2;
	uint16_t CANin_3;
	uint16_t CANin_4;
	uint16_t CANin_5;
	uint16_t CANin_6;
	uint16_t CANin_7;
	uint16_t CANin_8;
	uint16_t CANin_9;
	uint16_t CANin_10;
	uint16_t CANin_11;
	uint16_t CANin_12;
	uint16_t CANin_13;
	uint16_t CANin_14;
	uint16_t CANin_15;
	uint16_t CANin_16;
	uint8_t tpsADC;
	uint8_t getNextError;
	uint8_t launchCorrection;
};

statuses currentStatus;

static uint32_t oldtime=millis();   // for the timeout
byte SpeedyResponse[100]; //The data buffer for the serial3 data. This is longer than needed, just in case
byte ByteNumber;  // pointer to which byte number we are reading currently
byte rpmLSB;   // Least significant byte for RPM message
byte rpmMSB;  // Most significant byte for RPM message
byte pwLSB;   // Least significant byte for PW message
byte pwMSB;  // Most significant byte for PW message
byte CEL;   //timer for how long CEL light be kept on
uint32_t updatePW;
byte odometerLSB;
byte odometerMSB;
byte FuelLevel;
byte OutsideTemp;
byte vssCanLSB;
byte vssCanMSB;
int CLT;   // to store coolant temp
unsigned int RPM,PW,PWcount;   //RPM and PW from speeduino
byte TPS,tempLight;   //TPS value and overheat light on/off
bool data_error; //indicator for the data from speeduino being ok.
uint8_t data[255]; //For DS2 data

#if ((STM32_CORE_VERSION_MINOR<=8) & (STM32_CORE_VERSION_MAJOR==1))
void SendData(HardwareTimer*){void SendData();}
#endif
 
void setup(){
  pinMode(pin, OUTPUT);
  Serial3.begin(115200);  // baudrate for Speeduino is 115200
  Serial.begin(115200); // for debugging
  #ifdef DS2_ENABLE
  Serial2.begin(9600, SERIAL_8E1);
  Serial2.setTimeout(ISO_TIMEOUT);
  #endif
  
  Can1.begin();
  Can1.setBaudRate(500000);

  CAN_msg_RPM.len = 8; //8 bytes in can message
  CAN_msg_CLT_TPS.len = 7;
  CAN_msg_MPG_CEL.len = 4;
  CAN_msg_RPM.id = 0x316; //CAN ID for RPM message is 0x316
  CAN_msg_CLT_TPS.id = 0x329; //CAN ID for CLT and TSP message is 0x329
  CAN_msg_MPG_CEL.id = 0x545; //CAN ID for fuel consumption and CEl light is 0x545

  // send this message to get rid of EML light and also set the static values for the message

  CAN_msg_MPG_CEL.buf[0]= 0x02;  //error State
  CAN_msg_MPG_CEL.buf[1]= 0x00;  //LSB Fuel consumption
  CAN_msg_MPG_CEL.buf[2]= 0x00;  //MSB Fuel Consumption
  CAN_msg_MPG_CEL.buf[3]= 0x00;  //Overheat light
  CAN_msg_MPG_CEL.buf[4]= 0x00; //not used, but set to zero just in case.
  CAN_msg_MPG_CEL.buf[5]= 0x00; //not used, but set to zero just in case.
  CAN_msg_MPG_CEL.buf[6]= 0x00; //not used, but set to zero just in case.
  CAN_msg_MPG_CEL.buf[7]= 0x00; //not used, but set to zero just in case.
  Can1.write(CAN_msg_MPG_CEL);

//set the static values for the other two messages
  CAN_msg_RPM.buf[0]= 0x00;
  CAN_msg_RPM.buf[1]= 0x07;
  CAN_msg_RPM.buf[4]= 0x65;
  CAN_msg_RPM.buf[5]= 0x12;
  CAN_msg_RPM.buf[6]= 0x00;
  CAN_msg_RPM.buf[7]= 0x62;

  CAN_msg_CLT_TPS.buf[0]= 0x07;
  CAN_msg_CLT_TPS.buf[2]= 0xB2;
  CAN_msg_CLT_TPS.buf[3]= 0x19;
  CAN_msg_CLT_TPS.buf[4]= 0x00;
  CAN_msg_CLT_TPS.buf[6]= 0x00;
  CAN_msg_CLT_TPS.buf[7]= 0x00; //not used, but set to zero just in case.
// zero the data to be sent by CAN bus, so it's not just random garbage

  CLT = 0;
  currentStatus.PW1 = 0;
  PWcount = 0;
  rpmLSB = 0;
  rpmMSB = 0;
  pwLSB = 0;
  pwMSB = 0;
  CEL = 0;
  tempLight = 0;
  data_error = false;

//setup hardwaretimer to send data for instrument cluster in 32Hz pace
#if defined(TIM1)
  TIM_TypeDef *Instance = TIM1;
#else
  TIM_TypeDef *Instance = TIM2;
#endif
  HardwareTimer *SendTimer = new HardwareTimer(Instance);
  SendTimer->setOverflow(32, HERTZ_FORMAT); // 32 Hz
#if ( STM32_CORE_VERSION_MAJOR < 2 )
  SendTimer->attachInterrupt(1, SendData);
  SendTimer->setMode(1, TIMER_OUTPUT_COMPARE);
#else //2.0 forward
  SendTimer->attachInterrupt(SendData);
#endif
  SendTimer->resume();
  
  requestData(); // all set. Start requesting data from speeduino
}

#ifdef DS2_ENABLE
uint32_t convertValue(float val, float mul = 1, float add = 0) {
  // uint32_t meaning we can set up to 4 bytes this way
  uint32_t convertedVal = (uint32_t) ((val - add)/mul); // we need to reverse what we do on logger side
  return convertedVal;
}

void addValToData(uint32_t val, uint8_t data[], uint8_t offset, uint8_t length = 1, bool reverseEndianess = false) {
  for(uint8_t i = 0; i < length; i++) {
    if(reverseEndianess) data[offset + i] = ((uint8_t *)&val)[i];
    else data[offset + i] = ((uint8_t *)&val)[3-i];
  }
}

void sendReply(uint8_t data[]) {
  data[0] = 0x12; // Not needed as our data already have that
  data[1] = 38; // ms42 response lenght is 38 (26 hex)
  data[2] = 0xA0; //Ack
  uint8_t offset = 3;

  // Here is where payload starts:
  float tempvalue;
  uint32_t valueToSend;
  uint8_t valueOffset;
  uint8_t valueLength;
  
  //RPM
  valueOffset = 0;
  valueLength = 2;
  valueToSend = currentStatus.RPM;
  addValToData(valueToSend, data, valueOffset + offset, valueLength);
  
  //VSS
  valueOffset = 2;
  valueLength = 1;
  valueToSend = ((vssCanMSB << 8) | (vssCanLSB));
  addValToData(valueToSend, data, valueOffset + offset, valueLength);
  
  //TPS
  tempvalue = currentStatus.TPS;
  valueOffset = 4;
  valueLength = 1;
  valueToSend = convertValue(tempvalue, 0.390625);
  addValToData(valueToSend, data, valueOffset + offset, valueLength);
  
  //MAF
  tempvalue = currentStatus.VE;
  valueOffset = 5;
  valueLength = 2;
  valueToSend = convertValue(tempvalue, 0.25);
  addValToData(valueToSend, data, valueOffset + offset, valueLength);
  
  //IAT
  tempvalue = currentStatus.IAT;
  valueOffset = 7;
  valueLength = 1;
  valueToSend = convertValue(tempvalue, 0.75, -8);
  addValToData(valueToSend, data, valueOffset + offset, valueLength);
  
  //CLT
  tempvalue = currentStatus.CLT;
  valueOffset = 8;
  valueLength = 1;
  valueToSend = convertValue(tempvalue, 0.75, -8);
  addValToData(valueToSend, data, valueOffset + offset, valueLength);
  
  //Oil temp
  tempvalue = CANin_1;
  valueOffset = 9;
  valueLength = 1;
  valueToSend = convertValue(tempvalue, 0.75, -8);
  addValToData(valueToSend, data, valueOffset + offset, valueLength);
  
  //Ignition Angle
  tempvalue = currentStatus.advance;
  valueOffset = 11;
  valueLength = 1;
  valueToSend = convertValue(tempvalue, -0.375, 72);
  addValToData(valueToSend, data, valueOffset + offset, valueLength);
  
  //IPW
  tempvalue = currentStatus.PW1;
  valueOffset = 12;
  valueLength = 2;
  valueToSend = convertValue(tempvalue, 0.04);
  addValToData(valueToSend, data, valueOffset + offset, valueLength);

  //ICV?
  tempvalue = currentStatus.idleLoad;
  valueOffset = 14;
  valueLength = 2;
  valueToSend = convertValue(tempvalue, 0.001526);
  addValToData(valueToSend, data, valueOffset + offset, valueLength);
  
  //ICV duty
  tempvalue = currentStatus.idleLoad;
  valueOffset = 16;
  valueLength = 2;
  valueToSend = convertValue(tempvalue, 0.001526);
  addValToData(valueToSend, data, valueOffset + offset, valueLength);
  
  //Battery Voltage
  tempvalue = currentStatus.battery10;
  valueOffset = 20;
  valueLength = 1;
  valueToSend = convertValue(tempvalue, 0.1015625);
  addValToData(valueToSend, data, valueOffset + offset, valueLength);

  //Lambda Int 1
  tempvalue = currentStatus.egoCorrection;
  valueOffset = 21;
  valueLength = 2;
  valueToSend = convertValue(tempvalue, 0.0015258789, -50);
  addValToData(valueToSend, data, valueOffset + offset, valueLength);
  
  //Lambda Int 2
  tempvalue = 0;
  valueOffset = 23;
  valueLength = 2;
  valueToSend = convertValue(tempvalue, 0.0015258789, -50);
  addValToData(valueToSend, data, valueOffset + offset, valueLength);

  //Load
  tempvalue = currentStatus.MAP;
  valueOffset = 29;
  valueLength = 2;
  valueToSend = convertValue(tempvalue, 0.021);
  addValToData(valueToSend, data, valueOffset + offset, valueLength);
  
  //Knock Voltage 1
  tempvalue = 0;
  valueOffset = 31;
  valueLength = 1;
  valueToSend = convertValue(tempvalue, 0.01952);
  addValToData(valueToSend, data, valueOffset + offset, valueLength);
  
  //Knock Voltage 2
  tempvalue = 0;
  valueOffset = 32;
  valueLength = 1;
  valueToSend = convertValue(tempvalue, 0.01952);
  addValToData(valueToSend, data, valueOffset + offset, valueLength);
  
  // Checksum
  uint8_t checksum = 0;
  for(uint8_t i = 0; i < data[1]-1; i++) {
    checksum ^= data[i]; // Simple XOR checksum
  }
  data[data[1]-1] = checksum;
  DS2.sendCommand(data);
  DS2.newCommand(); // we clear RX on our side so we don't read the stuff that we sent out
}

void sendEcuId(uint8_t data[]) {
  data[0] = 0x12; // Not really needed as our data already have this
  data[1] = 11;   // response length
  data[2] = 0xA0; //Ack
  //Ecu Id in ASCII, we use ms42 C6-SW version id here
  data[3] = 37;  // 7
  data[4] = 35;  // 5
  data[5] = 30;  // 0
  data[6] = 30;  // 0
  data[7] = 32;  // 2
  data[8] = 35;  // 5
  data[9] = 35;  // 5
  
    // Checksum
  uint8_t checksum = 0;
  for(uint8_t i = 0; i < data[1]-1; i++) {
    checksum ^= data[i]; // Simple XOR checksum
  }
  data[data[1]-1] = checksum;
  DS2.sendCommand(data);
  DS2.newCommand(); // we clear RX on our side so we don't read the stuff that we sent out
}
#endif

void requestData() {
  Serial3.write("A"); //Send A to request real time data
}

void readCanMessage() {
  //worg in progress
  if ( CAN_inMsg.id == 0x613 )
  {
    odometerLSB = CAN_inMsg.buf[0];
    odometerMSB = CAN_inMsg.buf[1];
    //Serial.print ("Odometer: "); Serial.println (odometerLSB + (odometerMSB << 8));
    FuelLevel = CAN_inMsg.buf[2];
    //Serial.print ("Fuel level: "); Serial.println (FuelLevel);
  }
  if ( CAN_inMsg.id == 0x615 )
  {
    OutsideTemp = CAN_inMsg.buf[3];
    //Serial.print ("Outside temp: "); Serial.println (OutsideTemp);
  }
  if ( CAN_inMsg.id == 0x153 )
  {
    vssCanLSB = CAN_inMsg.buf[1];
    vssCanMSB = CAN_inMsg.buf[2];
  }
}

void SendData()   // Send can messages in 32Hz phase from timer interrupt. This is important to make cluster work smoothly
{
  CAN_msg_RPM.buf[2]= rpmLSB; //RPM LSB
  CAN_msg_RPM.buf[3]= rpmMSB; //RPM MSB
  if (Can1.write(CAN_msg_RPM)) {
    digitalWrite(pin, !digitalRead(pin)); // Just to see with internal led that CAN messages are being sent
  }
  //Send CLT and TPS
  
  CAN_msg_CLT_TPS.buf[1]= CLT; //Coolant temp
  CAN_msg_CLT_TPS.buf[5]= TPS; //TPS value.
  Can1.write(CAN_msg_CLT_TPS);

  // Send fuel consumption and error lights
  if (CEL < 120){  
    if (CEL < 60){
      CAN_msg_MPG_CEL.buf[0]= 0x12;  // keep CEL And EML on for about 2 seconds
    }
    else{
      CAN_msg_MPG_CEL.buf[0]= 0x02;  // keep CEL on for about 4 seconds
      }
    CEL++;
    }
  else{
    CAN_msg_MPG_CEL.buf[0]= 0x00;  //CEL off
  }
  updatePW = updatePW + ( currentStatus.PW1 * (RPM/1000) );
  PWcount = (updatePW/5500); // fuel consumption is measured by rate of change in instrument cluster. So PW counter is increased by steps of one. And rate of update depends on PW from speeduino. This isn't yet working correctly.
    if (PWcount == 0xFFFF)
    {
      PWcount = 0;
    }
  pwMSB = PWcount >> 8;  //split to high and low byte
  pwLSB = PWcount;
  CAN_msg_MPG_CEL.buf[1]= pwLSB;  //LSB Fuel consumption
  CAN_msg_MPG_CEL.buf[2]= pwLSB;  //MSB Fuel Consumption
  CAN_msg_MPG_CEL.buf[3]= tempLight ;  //Overheat light
  Can1.write(CAN_msg_MPG_CEL);
}

//display the needed values in serial monitor for debugging
void displayData(){
  Serial.print ("RPM-"); Serial.print (RPM); Serial.print("\t");
  Serial.print ("PW-"); Serial.print (PW); Serial.print("\t");
  Serial.print ("PWcount-"); Serial.print (PWcount); Serial.print("\t");
  Serial.print ("CLT-"); Serial.print (CLT); Serial.print("\t");
  Serial.print ("TPS-"); Serial.print (TPS); Serial.println("\t");

}

void processData(){   // necessary conversion for the data before sending to CAN BUS
  unsigned int tempRPM;
  data_error = false; // set the received data as ok

  if (SpeedyResponse[0] != 65)  //The first data received should be A (65 is ascii)
    {
      data_error = true; //data received is probaply corrupted, don't use it.
      Serial.print ("Not an A message");
    }
  currentStatus.secl = SpeedyResponse[1];
  currentStatus.status1 = SpeedyResponse[2];
  currentStatus.engine = SpeedyResponse[3];
  currentStatus.dwell = SpeedyResponse[4];
  currentStatus.MAP = ((SpeedyResponse [6] << 8) | (SpeedyResponse [5]));
  currentStatus.IAT = SpeedyResponse[7];
  currentStatus.CLT = SpeedyResponse[8];
  currentStatus.batCorrection = SpeedyResponse[9];
  currentStatus.battery10 = SpeedyResponse[10];
  currentStatus.O2 = SpeedyResponse[11];
  currentStatus.egoCorrection = SpeedyResponse[12];
  currentStatus.iatCorrection = SpeedyResponse[13];
  currentStatus.wueCorrection = SpeedyResponse[14];
  currentStatus.RPM = ((SpeedyResponse [16] << 8) | (SpeedyResponse [15])); // RPM low & high (Int) TBD: probaply no need to split high and low bytes etc. this could be all simpler
  currentStatus.AEamount = SpeedyResponse[17];
  currentStatus.corrections = SpeedyResponse[18];
  currentStatus.VE = SpeedyResponse[19];
  currentStatus.afrTarget = SpeedyResponse[20];
  currentStatus.PW1 = ((SpeedyResponse [22] << 8) | (SpeedyResponse [21])); // PW low & high (Int) TBD: probaply no need to split high and low bytes etc. this could be all simpler
  currentStatus.TPS = SpeedyResponse[25];

  // check if received values makes sense and convert those if all is ok.
  if (currentStatus.RPM < 8000 && data_error == false)  // the engine will not probaply rev over 8000 RPM
  {
    tempRPM = currentStatus.RPM * 6.4; // RPM conversion factor for e46/e39 cluster
    rpmMSB = tempRPM >> 8;  //split to high and low byte
    rpmLSB = tempRPM;
  }
  else
  {
    data_error = true; //data received is probaply corrupted, don't use it.
    Serial.print ("Error. RPM Received:"); Serial.print (RPM); Serial.print("\t");
  }

  if (currentStatus.CLT < 182 && data_error == false)  // 142 degrees Celcius is the hottest temp that fits to the conversion. 
  {
    CLT = (currentStatus.CLT -40)*4/3+64;  // CLT conversion factor for e46/e39 cluster
    if (currentStatus.CLT > 160) {
      tempLight = 0x8;  // turn on engine temp warning light after 120 degrees Celcius
    }
    else
    {
      tempLight = 0x0;  // turn engine temp warning light off
    }
  }
  else
  {
    data_error = true;//data received is probaply corrupted, don't use it.
    Serial.print ("Error. CLT received:"); Serial.print (currentStatus.CLT); Serial.print("\t");
  }

  if (currentStatus.TPS < 101 && data_error == false)  //TPS values can only be from 0-100
  {
    TPS = map(currentStatus.TPS, 0, 100, 1, 254); //0-100 TPS value mapped to 0x01 to 0xFE range.
  }
  else
  {
    data_error = true; //data received is probaply corrupted, don't use it.
    Serial.print ("Error. TPS received:"); Serial.print (currentStatus.TPS); Serial.print("\t");
  }
}

void loop() {
  if (Serial3.available () > 0) {  // read bytes from serial3
    SpeedyResponse[ByteNumber ++] = Serial3.read();
  }
  if (ByteNumber > (75)){        // After 75 bytes all the data from speeduino has been received so time to process it (A + 74 databytes)
    oldtime = millis();          // All ok. zero out timeout calculation
    ByteNumber = 0;              // zero out the byte number pointer
    processData();               // do the necessary processing for received data
    if (data_error) {               // while processing the data, there has been problem, so read everything from serial buffer and start over
      Serial.println ("Speeduino data error!");
      while (Serial3.available() ) {
        Serial3.read();
      }
    }
    else {
      displayData();             // only required for debugging
    }
    requestData();               //restart data reading
  }
  if ( (millis()-oldtime) > 500) { // timeout if for some reason reading serial3 fails
    oldtime = millis();
    ByteNumber = 0;                // zero out the byte number pointer
    Serial.println ("Timeout from speeduino!");
    requestData();                //restart data reading
  }
  //we can also read stuff back from instrument cluster
  while (Can1.read(CAN_inMsg) ) 
  {
    readCanMessage();
  }
  // Read DS2 command and respond to it
  if(DS2.readCommand(data)) {
    uint8_t len = data[1];
    switch(data[2]) {
      case 0x0B:
        if(data[3] == 0x03) sendReply(data);
        break;
      case 0x00:
        if(data[3] == 0x16) sendEcuId(data);
        break;
      default: break;
    }
  }
}
