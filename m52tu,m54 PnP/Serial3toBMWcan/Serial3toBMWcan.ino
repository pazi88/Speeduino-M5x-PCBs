// This code is meant to read real time data from Speeduino EFI using serial3 connection in speeduino and convert that to CAN messages for BMW e39/e46 instrument clusters
// The hardware that the code is meant to be used is STM32F103C8T6 STM32 Development Board (BluePill) with MCP2551 transceiver.
// Created by pazi88 and there is no guarantee at all that any of this will work.
// Use Platform IO in VScode to compile this. The platformio.ini is at the folder above this. Open that folder to your workspace in PIO.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.

#define REV1_5  // 1.5 board revision has new features that work only in that in newer. If you compile this to older boards, comment out this to avoid possible problems.
#ifdef REV1_5
  #include "DS2.h"
  #include <MFL.h>
  #define pin_53  PA1  // Connected to Arduino Mega pin 53 (SS0 in schema)
  #define pin_49  PC15  // Connected to Arduino Mega pin 49 (SS1 in schema), used for AC idle up
  #define Fan_pin  PA15  // PWM fan output on ULN
  #define AC_pin  PB9  // AC compressor output on ULN
  DS2 DS2(Serial2);
#endif

#include "STM32_CAN.h"

//Bitsetting macros
#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))
#define BIT_CHECK(var,pos) !!((var) & (1U<<(pos)))
#define BIT_TOGGLE(var,pos) ((var)^= 1UL << (pos))
#define BIT_WRITE(var, pos, bitvalue) ((bitvalue) ? BIT_SET((var), (pos)) : bitClear((var), (pos)))

#define NOTHING_RECEIVED        0
#define R_MESSAGE               1
#define A_MESSAGE               2
#define PWM_MESSAGE             3

//Stock M52TU injectors scaling. Adjust this to trim fuel consumption. (injector size affects this)
//Bosch 0280155831 19000
#define PW_ADJUST           25000

#define SerialUpdateRate 30   // 30 Hz rate limit to update secondary serial data from speeduino
#define PWMFanFrequency 100   // 100 Hz Frequency for the BMW PWM fan
#define ClusterUpdateRate 50  // 50 Hz Frequency for the cars instrument cluster

#define BIT_LV_REQ_TCO_L       5  // Request For Lowering Cooling Temp (c_tco_bol_ect)
#define BIT_LV_ACCIN           6  // Air Conditioning Compressor Status (0=off, 1=on)
#define BIT_LV_ACIN            7  // Air Conditioning Request (0=off, 1=on)

static CAN_message_t CAN_msg_RPM;
static CAN_message_t CAN_msg_CLT_TPS;
static CAN_message_t CAN_msg_MPG_CEL;
static CAN_message_t CAN_inMsg;

STM32_CAN Can1( CAN1, DEF, RX_SIZE_128, TX_SIZE_64 );

// This struct gathers data read from speeduino. This is really just direct copy of what speeduino has internally
struct statuses {
  uint8_t secl; // secl is simply a counter that increments each second.
  uint8_t status1; // status1 Bitfield, inj1Status(0), inj2Status(1), inj3Status(2), inj4Status(3), DFCOOn(4), boostCutFuel(5), toothLog1Ready(6), toothLog2Ready(7)
  uint8_t engine; // Engine Status Bitfield, running(0), crank(1), ase(2), warmup(3), tpsaccaen(4), tpsacden(5), mapaccaen(6), mapaccden(7)
  uint8_t dwell; // Dwell in ms * 10
  uint16_t MAP; // 2 bytes for MAP
  uint8_t IAT;
  uint8_t CLT;
  uint8_t batCorrection; // Battery voltage correction (%)
  uint8_t battery10; // battery voltage
  uint8_t O2; // O2
  uint8_t egoCorrection; // Exhaust gas correction (%)
  uint8_t iatCorrection; // Air temperature Correction (%)
  uint8_t wueCorrection; // Warmup enrichment (%)
  uint16_t RPM; // rpm
  uint8_t AEamount; // acceleration enrichment (%)
  uint8_t corrections; // Total GammaE (%)
  uint8_t VE; // Current VE 1 (%)
  uint8_t afrTarget;
  uint16_t PW1; // Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
  uint8_t tpsDOT; // TPS DOT
  int8_t advance;
  uint8_t TPS; // TPS (0% to 100%)
  uint16_t loopsPerSecond;
  uint16_t freeRAM;
  uint8_t boostTarget; // boost target divided by 2 to fit in a byte
  uint8_t boostDuty;
  uint8_t spark; // Spark related bitfield, launchHard(0), launchSoft(1), hardLimitOn(2), softLimitOn(3), boostCutSpark(4), error(5), idleControlOn(6), sync(7)
  uint16_t rpmDOT;
  uint8_t ethanolPct; // Flex sensor value (or 0 if not used)
  uint8_t flexCorrection; // Flex fuel correction (% above or below 100)
  uint8_t flexIgnCorrection; // Ignition correction (Increased degrees of advance) for flex fuel
  uint8_t idleLoad;
  uint8_t testOutputs; // testEnabled(0), testActive(1)
  uint8_t O2_2; // O2
  uint8_t baro; // Barometer value
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
uint8_t SpeedyResponse[100]; //The data buffer for the serial3 data. This is longer than needed, just in case
uint8_t rpmLSB;   // Least significant byte for RPM message
uint8_t rpmMSB;  // Most significant byte for RPM message
uint8_t pwLSB;   // Least significant byte for PW message
uint8_t pwMSB;  // Most significant byte for PW message
uint8_t CEL;   //timer for how long CEL light be kept on
uint32_t updatePW;
uint8_t odometerLSB;
uint8_t odometerMSB;
uint8_t FuelLevel;
uint8_t ambientTemp;
int CLT; // to store coolant temp
uint32_t PWcount;
uint8_t TPS,tempLight; // TPS value and overheat light on/off
bool data_error; //indicator for the data from speeduino being ok.
bool responseSent; // to keep track if we have responded to data request or not.
bool newData; // This tells if we have new data available from speeduino or not.
bool ascMSG; // ASC message received.
bool doRequest; // when true, it's ok to reques more data from speeduino serial
uint8_t rRequestCounter; // only request PWM fan duty from speeduino once in a second
uint8_t PWMfanDuty; // PWM fan duty
uint8_t data[255]; // For DS2 data
uint8_t SerialState,canin_channel,currentCommand;
uint16_t CanAddress,runningClock;
uint16_t VSS,VSS1,VSS2,VSS3,VSS4;
uint8_t MSGcounter; //this keeps track of which multiplexed info is sent in 0x329 byte 0
uint8_t radOutletTemp;
uint8_t oilTemp;
uint8_t acBitfield;
uint8_t acBitfield2;
uint8_t eFanBitfield;

// define hardwaretimers
TIM_TypeDef *Instance1 = TIM1;
TIM_TypeDef *Instance2 = TIM3;
#ifdef REV1_5
TIM_TypeDef *Instance3 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(Fan_pin), PinMap_PWM);
uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(Fan_pin), PinMap_PWM));
HardwareTimer *PWMFanTimer = new HardwareTimer(Instance3);
#endif
HardwareTimer *SendTimer = new HardwareTimer(Instance1);
HardwareTimer *requestTimer = new HardwareTimer(Instance2);
 
void requestData() {
  if (doRequest){
    Serial3.write("A"); // Send A to request real time data
    doRequest = false;
  }
  #ifdef REV1_5
  // The PWM fan duty is only requested once in a second. This doesn't need to be that frequently updated.
  if (rRequestCounter >= SerialUpdateRate){
    Serial3.write("r");  // Send r to request PWM fan duty
    Serial3.write(0xAA); // $tsCanId placeholder
    Serial3.write(0x30); // Send output channels command 0x30
    Serial3.write(121);  // LSB offset for receiving PWM fan duty.
    Serial3.write(0);    // MSB offset for receiving PWM fan duty.
    Serial3.write(1);    // LSB length for receiving PWM fan duty.
    Serial3.write(0);    // MSB length for receiving PWM fan duty.
    rRequestCounter = 0;
  }
  else {
    rRequestCounter++;
  }
  #endif
}

void SendData()   // Send can messages in 50Hz phase from timer interrupt. This is important to be high enough Hz rate to make cluster work smoothly.
{
  if (ascMSG) {
    CAN_msg_RPM.buf[0]= 0x05;
  }
  else {
    CAN_msg_RPM.buf[0]= 0x01;
  }	
  CAN_msg_RPM.buf[2]= rpmLSB; // RPM LSB
  CAN_msg_RPM.buf[3]= rpmMSB; // RPM MSB
  if ( Can1.write(CAN_msg_RPM) ){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Just to see with internal led that CAN messages are being sent
  }
  //Send CLT and TPS
  
  CAN_msg_CLT_TPS.buf[1]= CLT; // Coolant temp
  CAN_msg_CLT_TPS.buf[5]= TPS; // TPS value.
    //Multiplexed Information in byte0
  switch (MSGcounter) {
  case 0: //CAN_LEVEL
    CAN_msg_CLT_TPS.buf[0]= 0x11;
    break;
  case 1: //OBD_STEUER
    if (currentStatus.RPM < 400)
    {
      CAN_msg_CLT_TPS.buf[0]= 0x80;
    }
    else
    {
      CAN_msg_CLT_TPS.buf[0]= 0x86;
    }
    break;
  case 2: //MD_NORM
    CAN_msg_CLT_TPS.buf[0]= 0xD9;
    break;
  default:
    CAN_msg_CLT_TPS.buf[0]= 0x11;
    break;
}
  Can1.write(CAN_msg_CLT_TPS);

  // Send fuel consumption and error lights
  if (CEL < 200){  
    if (CEL < 100){
      CAN_msg_MPG_CEL.buf[0]= 0x12;  // keep CEL And EML on for about 2 seconds
    }
    else{
      CAN_msg_MPG_CEL.buf[0]= 0x02;  // keep CEL on for about 4 seconds
      }
    CEL++;
    }
  else{
    CAN_msg_MPG_CEL.buf[0]= 0x00;  // CEL off
  }
  // This updates the fuel consumption counter. It's how much fuel is injected to engine, so PW and RPM affects it.
  updatePW = updatePW + ( currentStatus.PW1 * (currentStatus.RPM/10) );
  // We adjust the counter reading so that we get correct values sent to cluster. PW_ADJUST is just trial and error value. 
  PWcount = updatePW / PW_ADJUST;
  // Fuel consumption counter is 2-bytes so if the current value is higher than that, we roll over the counter.
  if (PWcount > 0xFFFF)
  {
    PWcount = PWcount - 0xFFFF;
    updatePW = updatePW - (0xFFFF * PW_ADJUST);
  }
  pwMSB = highByte(uint16_t(PWcount));  // split to high and low byte
  pwLSB = lowByte(uint16_t(PWcount));
  CAN_msg_MPG_CEL.buf[1]= pwLSB;  // LSB Fuel consumption
  CAN_msg_MPG_CEL.buf[2]= pwMSB;  // MSB Fuel Consumption
  CAN_msg_MPG_CEL.buf[3]= tempLight ;  // Overheat light
  Can1.write(CAN_msg_MPG_CEL);
  MSGcounter++;
  if (MSGcounter >= 3)
  {
    MSGcounter = 0;
  }
}

void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Serial3.begin(115200);  // baudrate for Speeduino is 115200
  Serial.begin(115200); // for debugging
  #ifdef REV1_5
  Serial2.begin(9600, SERIAL_8E1);
  Serial2.setTimeout(ISO_TIMEOUT);
  setupMFL();
  pinMode(pin_53, OUTPUT);
  pinMode(pin_49, OUTPUT);
  pinMode(AC_pin, OUTPUT);
  #endif
  
  doRequest = false;
  rRequestCounter = 0;
  Can1.begin();
  Can1.setBaudRate(500000);
  Can1.setMBFilterProcessing( MB0, 0x153, 0x1FFFFFFF );
  Can1.setMBFilterProcessing( MB1, 0x613, 0x1FFFFFFF );
  Can1.setMBFilterProcessing( MB2, 0x615, 0x1FFFFFFF );
  Can1.setMBFilterProcessing( MB3, 0x1F0, 0x1FFFFFFF );

  CAN_msg_RPM.len = 8; // 8 bytes in can message
  CAN_msg_CLT_TPS.len = 7;
  CAN_msg_MPG_CEL.len = 4;
  CAN_msg_RPM.id = 0x316; // CAN ID for RPM message is 0x316
  CAN_msg_CLT_TPS.id = 0x329; // CAN ID for CLT and TSP message is 0x329
  CAN_msg_MPG_CEL.id = 0x545; // CAN ID for fuel consumption and CEl light is 0x545

  // send this message to get rid of EML light and also set the static values for the message
  CAN_msg_MPG_CEL.buf[0]= 0x02;  // error State
  CAN_msg_MPG_CEL.buf[1]= 0x00;  // LSB Fuel consumption
  CAN_msg_MPG_CEL.buf[2]= 0x00;  // MSB Fuel Consumption
  CAN_msg_MPG_CEL.buf[3]= 0x00;  // Overheat light
  CAN_msg_MPG_CEL.buf[4]= 0x00; // not used, but set to zero just in case.
  CAN_msg_MPG_CEL.buf[5]= 0x00; // not used, but set to zero just in case.
  CAN_msg_MPG_CEL.buf[6]= 0x00; // not used, but set to zero just in case.
  CAN_msg_MPG_CEL.buf[7]= 0x00; // not used, but set to zero just in case.
  Can1.write(CAN_msg_MPG_CEL);

  // set the static values for the other two messages
  CAN_msg_RPM.buf[0]= 0x01;  //bitfield, Bit0 = 1 = terminal 15 on detected, Bit2 = 1 = 1 = the ASC message ASC1 was received within the last 500 ms and contains no plausibility errors
  CAN_msg_RPM.buf[1]= 0x0C;  //Indexed Engine Torque in % of C_TQ_STND TBD do torque calculation!!
  CAN_msg_RPM.buf[4]= 0x0C;  //Indicated Engine Torque in % of C_TQ_STND TBD do torque calculation!! Use same as for byte 1
  CAN_msg_RPM.buf[5]= 0x15;  //Engine Torque Loss (due to engine friction, AC compressor and electrical power consumption)
  CAN_msg_RPM.buf[6]= 0x00;  //not used
  CAN_msg_RPM.buf[7]= 0x35;  //Theorethical Engine Torque in % of C_TQ_STND after charge intervention

  CAN_msg_CLT_TPS.buf[0]= 0x11;  //Multiplexed Information
  CAN_msg_CLT_TPS.buf[2]= 0xB2;  //CLT temp
  CAN_msg_CLT_TPS.buf[3]= 0x00;  //Baro
  CAN_msg_CLT_TPS.buf[4]= 0x08;  //bitfield, Bit0 = 0 = Clutch released, Bit 3 = 1 = engine running
  CAN_msg_CLT_TPS.buf[6]= 0x00;  //TPS_VIRT_CRU_CAN (Not used)
  CAN_msg_CLT_TPS.buf[7]= 0x00;  //not used, but set to zero just in case.

  // Start with sensible values for some of these variables.
  CLT = 60;
  currentStatus.PW1 = 0;
  updatePW = 0;
  rpmLSB = 0;
  rpmMSB = 0;
  pwLSB = 0;
  pwMSB = 0;
  CEL = 0;
  tempLight = 0;
  SerialState = NOTHING_RECEIVED;
  data_error = false;
  responseSent = false;
  newData = false;
  MSGcounter = 0;
  ascMSG = false;
  radOutletTemp = 0;
  oilTemp = 0;
  PWMfanDuty = 10; // minimum valid duty is 10%
  acBitfield = 0;
  acBitfield2 = 0;
  eFanBitfield = 0;

  // setup hardwaretimers
  requestTimer->setOverflow(SerialUpdateRate, HERTZ_FORMAT);
  SendTimer->setOverflow(ClusterUpdateRate, HERTZ_FORMAT);
  requestTimer->attachInterrupt(requestData);
  SendTimer->attachInterrupt(SendData);
  requestTimer->resume();
  SendTimer->resume();
#ifdef REV1_5
  PWMFanTimer->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, Fan_pin);
  PWMFanTimer->setOverflow(PWMFanFrequency, HERTZ_FORMAT);
  PWMFanTimer->setCaptureCompare(channel, 10, PERCENT_COMPARE_FORMAT); // 10%
  PWMFanTimer->resume();
#endif

  Serial.println ("Version date: 6.6.2023"); // To see from debug serial when is used code created.
  doRequest = true; // all set. Start requesting data from speeduino
  digitalWrite(AC_pin, LOW);
  digitalWrite(pin_49, LOW);
  digitalWrite(pin_53, LOW);
  rRequestCounter = SerialUpdateRate;
}

#ifdef REV1_5
uint32_t convertValue(float val, float mul = 1, float add = 0) {
  // uint32_t meaning we can set up to 4 bytes this way
  uint32_t convertedVal = (uint32_t) ((val - add)/mul); // we need to reverse what we do on logger side
  return convertedVal;
}

void addValToData(uint16_t val, uint8_t data[], uint8_t offset, uint8_t length = 1) {
  if(length == 1) {
    data[offset] = lowByte(val);
  }
  else if (length == 2) {
    data[offset] = highByte(val);
    data[offset + 1] = lowByte(val);
  }
}

void sendReply(uint8_t data[]) {
  data[0] = 0x12; // Not needed as our data already have that
  data[1] = 38;   // ms42 response lenght is 38 (26 hex)
  data[2] = 0xA0; // Ack
  uint8_t offset = 3; // payload starts after 3 initial bytes

  // Here is where payload starts:
  uint16_t valueToSend;
  
  // RPM
  //addValToData(valueToSend, data, valueOffset + offset, valueLength);
  addValToData(currentStatus.RPM, data, 0 + offset, 2);
  
  // VSS
  addValToData(VSS1, data, 2 + offset, 1);
  
  // TPS
  valueToSend = convertValue(currentStatus.TPS, 0.390625);
  addValToData(valueToSend, data, 4 + offset, 1);
  
  // MAF
  valueToSend = convertValue(currentStatus.VE, 0.25);
  addValToData(valueToSend, data, 5 + offset, 2);
  
  // IAT
  valueToSend = convertValue(currentStatus.IAT, 0.75, -8);
  addValToData(valueToSend, data, 7 + offset, 1);
  
  // CLT
  valueToSend = convertValue(currentStatus.CLT, 0.75, -8);
  addValToData(valueToSend, data, 8 + offset, 1);
  
  // Oil temp
  valueToSend = convertValue(currentStatus.CANin_1, 0.75, -8);
  addValToData(valueToSend, data, 9 + offset, 1);
  
  // Ignition Angle
  valueToSend = convertValue(currentStatus.advance, -0.375, 72);
  addValToData(valueToSend, data, 11 + offset, 1);
  
  // IPW
  valueToSend = convertValue((currentStatus.PW1 / 1000), 0.04);
  addValToData(valueToSend, data, 12 + offset, 2);

  // ICV?
  valueToSend = convertValue(currentStatus.idleLoad, 0.001526);
  addValToData(valueToSend, data, 14 + offset, 2);
  
  // ICV duty
  valueToSend = convertValue(currentStatus.idleLoad, 0.001526);
  addValToData(valueToSend, data, 16 + offset, 2);
  
  // Battery Voltage
  addValToData(currentStatus.battery10, data, 20 + offset, 1);

  // Lambda Int 1
  valueToSend = convertValue(currentStatus.egoCorrection, 0.0015258789, -50);
  addValToData(valueToSend, data, 21 + offset, 2);
  
  // Lambda Int 2
  valueToSend = convertValue(0, 0.0015258789, -50);
  addValToData(valueToSend, data, 23 + offset, 2);

  // Load
  valueToSend = convertValue(currentStatus.MAP, 0.021);
  addValToData(valueToSend, data, 29 + offset, 2);
  
  // Knock Voltage 1
  valueToSend = convertValue(0, 0.01952);
  addValToData(valueToSend, data, 31 + offset, 1);
  
  // Knock Voltage 2
  valueToSend = convertValue(0, 0.01952);
  addValToData(valueToSend, data, 32 + offset, 1);
  
  // Checksum
  uint8_t checksum = 0;
  for(uint8_t i = 0; i < data[1]-1; i++) {
    checksum ^= data[i]; // Simple XOR checksum
  }
  data[data[1]-1] = checksum;
  DS2.writeData(data);
  responseSent = true;
  newData = false; // we have now sent out the data we have received.
}

void sendSpeedyReply(uint8_t data[]) {
  data[0] = 0x12; // Not needed as our data already have that
  data[1] = 53;
  data[2] = 0xA0; // Ack
  uint8_t offset = 3; // payload starts after 3 initial bytes
  
  // Just put data from speedyresponse to K-line response
  for (uint8_t i = 0; i < 41; i++) {
    data[i + offset] = SpeedyResponse[i];
  }
  addValToData(radOutletTemp, data, 41 + offset, 1);
  addValToData(oilTemp, data, 42 + offset, 1);
  addValToData(VSS1, data, 43 + offset, 2);
  addValToData(VSS2, data, 45 + offset, 2);
  addValToData(VSS3, data, 47 + offset, 2);
  addValToData(VSS4, data, 49 + offset, 2);

  // Checksum
  uint8_t checksum = 0;
  for(uint8_t i = 0; i < data[1]-1; i++) {
    checksum ^= data[i]; // Simple XOR checksum
  }
  data[data[1]-1] = checksum;
  DS2.writeData(data);
  responseSent = true;
  newData = false; // we have now sent out the data we have received.
}

void sendEcuId(uint8_t data[]) {
  data[0] = 0x12; // Not really needed as our data already have this
  data[1] = 11;   // response length
  data[2] = 0xA0; // Ack
  // Ecu Id in ASCII, we use ms42 C6-SW version id here
  data[3] = 0x37;  // 7
  data[4] = 0x35;  // 5
  data[5] = 0x30;  // 0
  data[6] = 0x30;  // 0
  data[7] = 0x32;  // 2
  data[8] = 0x35;  // 5
  data[9] = 0x35;  // 5
  
    // Checksum
  uint8_t checksum = 0;
  for(uint8_t i = 0; i < data[1]-1; i++) {
    checksum ^= data[i]; // Simple XOR checksum
  }
  data[data[1]-1] = checksum;
  DS2.writeData(data);
  responseSent = true;
}
#endif

void readCanMessage() {
  switch (CAN_inMsg.id)
  {
    case 0x613:
      odometerLSB = CAN_inMsg.buf[0];
      odometerMSB = CAN_inMsg.buf[1];
      //Serial.print ("Odometer: "); Serial.println (odometerLSB + (odometerMSB << 8));
      FuelLevel = CAN_inMsg.buf[2];
      //Serial.print ("Fuel level: "); Serial.println (FuelLevel);
      runningClock = ((CAN_inMsg.buf[4] << 8) | (CAN_inMsg.buf[3]));
    break;
    case 0x615:
      ambientTemp = CAN_inMsg.buf[3];
      //Serial.print ("Outside temp: "); Serial.println (ambientTemp);
      acBitfield = CAN_inMsg.buf[0];
      acBitfield2 = CAN_inMsg.buf[4];
      eFanBitfield = CAN_inMsg.buf[1];
    break;
    case  0x153: 
      ascMSG = true;
      VSS = ((CAN_inMsg.buf[2] << 8) | (CAN_inMsg.buf[1]));
      // conversion (speeduino doesn't have internal conversion for CAN data, so we do it here)
      VSS = VSS >> 7; // divide by 128
      VSS = VSS - 2;
    break;
    case  0x1F0:
      VSS1 = ((CAN_inMsg.buf[1] << 8) | (CAN_inMsg.buf[0]));
      // conversion
      VSS1 = VSS1 >> 4; // divide by 16
      VSS1 = VSS1 - 2;
      VSS2 = ((CAN_inMsg.buf[3] << 8) | (CAN_inMsg.buf[2]));
      VSS2 = VSS2 >> 4;
      VSS2 = VSS2 - 2;
      VSS3 = ((CAN_inMsg.buf[5] << 8) | (CAN_inMsg.buf[4]));
      VSS3 = VSS3 >> 4;
      VSS3 = VSS3 - 2;
      VSS4 = ((CAN_inMsg.buf[7] << 8) | (CAN_inMsg.buf[6]));
      VSS4 = VSS4 >> 4;
      VSS4 = VSS4 - 2;
    break;
    default:
      // nothing to do here
    break;
  }
}

void SendDataToSpeeduino(){
  Serial3.write("G");                      // reply "G" cmd
  switch (CanAddress)
  {
    case 0x613:  // Odometer and fuel level
      Serial3.write(1);                        // send 1 to confirm cmd received and valid
      Serial3.write(canin_channel);            // confirms the destination channel
      Serial3.write(odometerLSB);              // write back the requested data
      Serial3.write(odometerMSB);
      Serial3.write(FuelLevel);
      Serial3.write(lowByte(runningClock));
      Serial3.write(highByte(runningClock));
      for (int i=0; i<3; i++) {                // Rest will be zero
        Serial3.write(0);
      }
    break;
    case 0x615:  // Ambient temp
      Serial3.write(1);                        // send 1 to confirm cmd received and valid
      Serial3.write(canin_channel);            // confirms the destination channel
      for (int i=0; i<3; i++) {
          Serial3.write(0);
      }
      Serial3.write(ambientTemp);              // write back the requested data
        for (int i=0; i<4; i++) {
          Serial3.write(0);
      }
    break;
    case  0x153:  // VSS
      Serial3.write(1);                        // send 1 to confirm cmd received and valid
      Serial3.write(canin_channel);            // confirms the destination channel
      Serial3.write(0);
      Serial3.write(lowByte(VSS));
      Serial3.write(highByte(VSS));
      for (int i=0; i<5; i++) {
          Serial3.write(0);
      }
    break;
    case  0x1F0:  // VSS for each invidual wheel
      Serial3.write(1);                        // send 1 to confirm cmd received and valid
      Serial3.write(canin_channel);            // confirms the destination channel               //write back the requested data
      Serial3.write(lowByte(VSS1));
      Serial3.write(highByte(VSS1));
      Serial3.write(lowByte(VSS2));
      Serial3.write(highByte(VSS2));
      Serial3.write(lowByte(VSS3));
      Serial3.write(highByte(VSS3));
      Serial3.write(lowByte(VSS4));
      Serial3.write(highByte(VSS4));
    break;
    default:
      Serial3.write(0);                        // send 0 to confirm cmd received but not valid
      Serial3.write(canin_channel);            // destination channel
      for (int i=0; i<8; i++) {                // we need to still write some crap as an response, or real time data reading will slow down significantly
          Serial3.write(0);
      }
      Serial.print ("Wrong CAN address");
    break;
  }
}

// display the needed values in serial monitor for debugging
void displayData(){
  Serial.print ("RPM-"); Serial.print (currentStatus.RPM); Serial.print("\t");
  Serial.print ("PW-"); Serial.print (currentStatus.PW1); Serial.print("\t");
  Serial.print ("PWcount-"); Serial.print (PWcount); Serial.print("\t");
  Serial.print ("CLT-"); Serial.print (CLT); Serial.print("\t");
  Serial.print ("TPS-"); Serial.print (TPS); Serial.println("\t");

}

void processData(){   // necessary conversion for the data before sending to CAN BUS
  unsigned int tempRPM;
  data_error = false; // set the received data as ok

  currentStatus.secl = SpeedyResponse[0];
  currentStatus.status1 = SpeedyResponse[1];
  currentStatus.engine = SpeedyResponse[2];
  currentStatus.dwell = SpeedyResponse[3];
  currentStatus.MAP = ((SpeedyResponse [5] << 8) | (SpeedyResponse [4]));
  currentStatus.IAT = SpeedyResponse[6];
  currentStatus.CLT = SpeedyResponse[7];
  currentStatus.batCorrection = SpeedyResponse[8];
  currentStatus.battery10 = SpeedyResponse[9];
  currentStatus.O2 = SpeedyResponse[10];
  currentStatus.egoCorrection = SpeedyResponse[11];
  currentStatus.iatCorrection = SpeedyResponse[12];
  currentStatus.wueCorrection = SpeedyResponse[13];
  currentStatus.RPM = ((SpeedyResponse [15] << 8) | (SpeedyResponse [14])); // RPM low & high (Int) TBD: probaply no need to split high and low bytes etc. this could be all simpler
  currentStatus.AEamount = SpeedyResponse[16];
  currentStatus.corrections = SpeedyResponse[17];
  currentStatus.VE = SpeedyResponse[18];
  currentStatus.afrTarget = SpeedyResponse[19];
  currentStatus.PW1 = ((SpeedyResponse [21] << 8) | (SpeedyResponse [20])); // PW low & high (Int) TBD: probaply no need to split high and low bytes etc. this could be all simpler
  currentStatus.tpsDOT = SpeedyResponse[22];
  currentStatus.advance = SpeedyResponse[23];
  currentStatus.TPS = SpeedyResponse[24];
  currentStatus.loopsPerSecond = ((SpeedyResponse [26] << 8) | (SpeedyResponse [25]));
  currentStatus.freeRAM = ((SpeedyResponse [28] << 8) | (SpeedyResponse [27]));
  currentStatus.boostTarget = SpeedyResponse[29]; // boost target divided by 2 to fit in a byte
  currentStatus.boostDuty = SpeedyResponse[30];
  currentStatus.spark = SpeedyResponse[31]; // Spark related bitfield, launchHard(0), launchSoft(1), hardLimitOn(2), softLimitOn(3), boostCutSpark(4), error(5), idleControlOn(6), sync(7)
  currentStatus.rpmDOT = ((SpeedyResponse [33] << 8) | (SpeedyResponse [32]));
  currentStatus.ethanolPct = SpeedyResponse[34]; // Flex sensor value (or 0 if not used)
  currentStatus.flexCorrection = SpeedyResponse[35]; // Flex fuel correction (% above or below 100)
  currentStatus.flexIgnCorrection = SpeedyResponse[36]; // Ignition correction (Increased degrees of advance) for flex fuel
  currentStatus.idleLoad = SpeedyResponse[37];
  currentStatus.testOutputs = SpeedyResponse[38]; // testEnabled(0), testActive(1)
  currentStatus.O2_2 = SpeedyResponse[39]; // O2
  currentStatus.baro = SpeedyResponse[40]; // Barometer value
  currentStatus.CANin_1 = ((SpeedyResponse [42] << 8) | (SpeedyResponse [41]));
  currentStatus.CANin_2 = ((SpeedyResponse [44] << 8) | (SpeedyResponse [43]));
  currentStatus.CANin_3 = ((SpeedyResponse [46] << 8) | (SpeedyResponse [45]));
  currentStatus.CANin_4 = ((SpeedyResponse [48] << 8) | (SpeedyResponse [47]));
  currentStatus.CANin_5 = ((SpeedyResponse [50] << 8) | (SpeedyResponse [49]));
  currentStatus.CANin_6 = ((SpeedyResponse [52] << 8) | (SpeedyResponse [51]));
  currentStatus.CANin_7 = ((SpeedyResponse [54] << 8) | (SpeedyResponse [53]));
  currentStatus.CANin_8 = ((SpeedyResponse [56] << 8) | (SpeedyResponse [55]));
  currentStatus.CANin_9 = ((SpeedyResponse [58] << 8) | (SpeedyResponse [57]));
  currentStatus.CANin_10 = ((SpeedyResponse [60] << 8) | (SpeedyResponse [59]));
  currentStatus.CANin_11 = ((SpeedyResponse [62] << 8) | (SpeedyResponse [61]));
  currentStatus.CANin_12 = ((SpeedyResponse [64] << 8) | (SpeedyResponse [63]));
  currentStatus.CANin_13 = ((SpeedyResponse [66] << 8) | (SpeedyResponse [65]));
  currentStatus.CANin_14 = ((SpeedyResponse [68] << 8) | (SpeedyResponse [67]));
  currentStatus.CANin_15 = ((SpeedyResponse [70] << 8) | (SpeedyResponse [69]));
  currentStatus.CANin_16 = ((SpeedyResponse [71] << 8) | (SpeedyResponse [71]));
  currentStatus.tpsADC = SpeedyResponse[73];

  // check if received values makes sense and convert those if all is ok.
  if (currentStatus.RPM < 8000 && data_error == false)  // the engine will not probaply rev over 8000 RPM
  {
    tempRPM = currentStatus.RPM * 6.4; // RPM conversion factor for e46/e39 cluster
    rpmMSB = tempRPM >> 8;  // split to high and low byte
    rpmLSB = tempRPM;
  }
  else
  {
    data_error = true; // data received is probaply corrupted, don't use it.
    Serial.print ("Error. RPM Received:"); Serial.print (currentStatus.RPM); Serial.print("\t");
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
    data_error = true;  // data received is probaply corrupted, don't use it.
    Serial.print ("Error. CLT received:"); Serial.print (currentStatus.CLT); Serial.print("\t");
  }

  if (currentStatus.TPS < 201 && data_error == false)  // TPS values can only be from 0-200 (previously this was from 0-100 on speeduino)
  {
    TPS = map(currentStatus.TPS, 0, 200, 0, 254); // 0-100 TPS value mapped to 0x00 to 0xFE range.
    newData = true; // we have now new data and it passes the checks.
  }
  else
  {
    data_error = true; // data received is probaply corrupted, don't use it.
    Serial.print ("Error. TPS received:"); Serial.print (currentStatus.TPS); Serial.print("\t");
  }
}

void HandleA()
{
  Serial.print ("A ");
  data_error = false;
  for (int i=0; i<75; i++) {
    SpeedyResponse[i] = Serial3.read();
    }
  processData();                  // do the necessary processing for received data
  //displayData();                  // only required for debugging
  doRequest = true;               // restart data reading
  oldtime = millis();             // zero the timeout
  SerialState = NOTHING_RECEIVED; // all done. We set state for reading what's next message.
}

void HandleR()
{
  Serial.println ("R ");
  byte tmp0;
  byte tmp1;
  canin_channel = Serial3.read();
  tmp0 = Serial3.read();  // read in lsb of source can address
  tmp1 = Serial3.read();  // read in msb of source can address
  CanAddress = tmp1<<8 | tmp0 ;
  SendDataToSpeeduino();  // send the data to speeduino
  SerialState = NOTHING_RECEIVED; // all done. We set state for reading what's next message.
}

void Handle_r()
{
#ifdef REV1_5
  uint8_t tmp_duty;
  Serial.println ("r ");
  data_error = false;
  if (Serial3.read() != 0x30){  // r-command always returns 0x30. Something is wrong if it doesn't.
    data_error = true;
  }
  else {
    tmp_duty = Serial3.read(); // read the fan duty.
    tmp_duty = tmp_duty / 2;   // speeduino range is 0-200. Change it to 0-100
    if (tmp_duty < 10){        // adjust the range to valid 10-90 for BMW fan
      tmp_duty = 10;
    }
    else if (tmp_duty > 90){
      tmp_duty = 90;
    }
    PWMfanDuty = tmp_duty;
    PWMFanTimer->setCaptureCompare(channel, tmp_duty, PERCENT_COMPARE_FORMAT); // 10%
    PWMFanTimer->resume();
  }
  SerialState = NOTHING_RECEIVED; // all done. We set state for reading what's next message.
#endif
}

void ReadSerial()
{
  currentCommand = Serial3.read();
  switch (currentCommand)
  {
    case 'A':  // Speeduino sends data in A-message
      SerialState = A_MESSAGE;
    break;
    case 'R':  // Speeduino requests data in R-message
      SerialState = R_MESSAGE;
    break;
    case 'r':  // Speeduino sends the Fan PWM duty in data in r-message, because we requested it.
      SerialState = PWM_MESSAGE;
    break;
    default:
      Serial.print ("Not an A or R message ");
      Serial.println (currentCommand);
    break;
  }
}

// main loop
void loop() {
  switch(SerialState) {
    case NOTHING_RECEIVED:
      if (Serial3.available() > 0) { ReadSerial(); }  // read bytes from serial3 to define what message speeduino is sending.
      break;
    case A_MESSAGE:
      if (Serial3.available() >= 74) { HandleA(); }  // read and process the A-message from serial3, when it's fully received.
      break;
    case R_MESSAGE:
      if (Serial3.available() >= 3) {  HandleR(); }  // read and process the R-message from serial3, when it's fully received.
      break;
    case PWM_MESSAGE:
      if (Serial3.available() >= 2) {  Handle_r(); }  // read and process the r-message from serial3, when it's fully received.
      break;
    default:
      break;
  }

  if ( (millis()-oldtime) > 500) { // timeout if for some reason reading serial3 fails
    oldtime = millis();
    Serial.println ("Timeout from speeduino!");
    doRequest = true;                // restart data reading
  }
// we can also read stuff back from instrument cluster
  while (Can1.read(CAN_inMsg) ) 
  {
    readCanMessage();
  }
#ifdef REV1_5
// see if there is commands available from K-line
  if( responseSent == false ){
    // commands are 4 bytes long, so we only start reading RX buffer, when whe have full command there.
    if( (DS2.available() >= 4) && newData ){ // we also want to have new updated data available from speeduino, or it's not worth sending anything to K-line.
      if(DS2.readCommand(data)){  //Read command will ensure it's own length and if checksum is ok.
        switch(data[2]) {
          case 0x0B:
            if(data[3] == 0x03) sendReply(data);
            break;
            if(data[3] == 0xFF) sendSpeedyReply(data);  // Special case to get whole unaltered speeduino response to be logged via K-line
            break;
          case 0x00:
            if(data[3] == 0x16) sendEcuId(data);
            break;
          case 0xA0:
            Serial.println("Error! DS2 Received Ack.");
            break;
          default:
            Serial.println("Not supported DS2 command");
          break;
        }
      }
    }
  }
  // if we have sent the response, we'll wait for the echo of to be filled in serial buffer and then we will just read it out to get rid of it.
  else if( responseSent == true ){
    if( DS2.available() >= DS2.getEcho() ){
      DS2.readCommand(data);
      responseSent = false; // there is no more echo on the RX buffer, so we are ready to read new command
    }
  }
  // Lets see if any of the cruise buttons have been pressed
  updateCruise();
  // Set input pin states on Speeduino based on cruise buttons.
  digitalWrite(pin_53, MFL_CRUISE_MINUS);
  // This doesn't have any real purpose, but lets light up cruise light in instrument cluster if cruise on/off button is pressed.
  if (MFL_CRUISE_IO) {
    CAN_msg_MPG_CEL.buf[0] = 0x8;
  }
  else {
    CAN_msg_MPG_CEL.buf[0] = 0x0;
  }
  
  // AC-stuff
  if ( (BIT_CHECK(acBitfield, BIT_LV_ACCIN)) || (BIT_CHECK(acBitfield2, BIT_LV_ACCIN)) ) {
    digitalWrite(AC_pin, HIGH); // turn on AC compressor
    digitalWrite(pin_49, HIGH); // request higher idle from speeduino
  }
  else {
    digitalWrite(AC_pin, LOW); // turn off AC compressor
    digitalWrite(pin_49, LOW); // normal idle
  }
#endif
}
