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

#ifdef ARDUINO_BLUEPILL_F103C8
HardwareSerial Serial3(USART3); //for some reason this isn't defined in arduino_core_stm32
#endif
#define pin  LED_BUILTIN

enum BITRATE{CAN_50KBPS, CAN_100KBPS, CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS};
typedef struct
{
  uint16_t id;
  uint8_t  data[8];
  uint8_t  len;
} CAN_msg_t;

typedef const struct
{
  uint8_t TS2;
  uint8_t TS1;
  uint8_t BRP;
} CAN_bit_timing_config_t;
CAN_bit_timing_config_t can_configs[6] = {{2, 13, 45}, {2, 15, 20}, {2, 13, 18}, {2, 13, 9}, {2, 15, 4}, {2, 15, 2}};

extern CAN_bit_timing_config_t can_configs[6];
static uint32_t oldtime=millis();   // for the timeout
byte SpeedyResponse[100]; //The data buffer for the serial3 data. This is longer than needed, just in case
byte ByteNumber;  // pointer to which byte number we are reading currently
byte rpmLSB;   // Least significant byte for RPM message
byte rpmMSB;  // Most significant byte for RPM message
byte pwLSB;   // Least significant byte for PW message
byte pwMSB;  // Most significant byte for PW message
byte CEL;   //timer for how long CEL light be kept on
byte readCLT; // CLT doesn't need to be updated very ofter so 
uint32_t updatePW;
int CLT;   // to store coolant temp
unsigned int RPM,PW,PWcount;   //RPM and PW from speeduino
byte TPS,tempLight;   //TPS value and overheat light on/off

CAN_msg_t CAN_msg_RPM;    // CAN message for RPM
CAN_msg_t CAN_msg_CLT_TPS;    // CAN message for CLT and TPS
CAN_msg_t CAN_msg_MPG_CEL;    // CAN message for fule consumption and CEL light

#if ((STM32_CORE_VERSION_MINOR<=8) & (STM32_CORE_VERSION_MAJOR==1))
void SendData(HardwareTimer*){void SendData();}
#endif

void CANInit(enum BITRATE bitrate) //CAN bus initialization
 {
    RCC->APB1ENR |= 0x2000000UL;      // Enable CAN clock 
    RCC->APB2ENR |= 0x1UL;            // Enable AFIO clock

    AFIO->MAPR   &= 0xFFFF9FFF;       // reset CAN remap
    AFIO->MAPR   |= 0x00000000;       //  et CAN remap, use PA11, PA12
 
    RCC->APB2ENR |= 0x4UL;            // Enable GPIOA clock (bit2 to 1)
    GPIOA->CRH   &= 0xFFF00FFF;
    GPIOA->CRH   |= 0xB8000UL;            // Configure PA11 and PA12
    GPIOA->ODR   |= 0x1000UL;
  
    CAN1->MCR = 0x51UL;                // Set CAN to initialization mode
     
    // Set bit rates 
    CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF)); 
    CAN1->BTR |=  (((can_configs[bitrate].TS2-1) & 0x07) << 20) | (((can_configs[bitrate].TS1-1) & 0x0F) << 16) | ((can_configs[bitrate].BRP-1) & 0x1FF);
 
// Configure Filters to default values
    CAN1->FMR  |=   0x1UL;                // Set to filter initialization mode
    CAN1->FMR  &= 0xFFFFC0FF;             // Clear CAN2 start bank
    CAN1->FMR  |= 0x1C << 8;              // Assign all filters to CAN1
    CAN1->FA1R &= ~(0x1UL);               // Deactivate filter 0
    CAN1->FS1R |=   0x1UL;                // Set first filter to single 32 bit configuration

 
    CAN1->sFilterRegister[0].FR1 = 0x0UL; // Set filter registers to 0
    CAN1->sFilterRegister[0].FR2 = 0x0UL; // Set filter registers to 0
    CAN1->FM1R &= ~(0x1UL);               // Set filter to mask mode
 
    CAN1->FFA1R &= ~(0x1UL);              // Apply filter to FIFO 0  
    CAN1->FA1R  |=   0x1UL;               // Activate filter 0
    
    CAN1->FMR   &= ~(0x1UL);              // Deactivate initialization mode
    CAN1->MCR   &= ~(0x1UL);              // Set CAN to normal mode 

    while (CAN1->MSR & 0x1UL); 
 }

void CANSend(CAN_msg_t* CAN_tx_msg) // CAN message send routine
 {
    volatile int count = 0;
     
    CAN1->sTxMailBox[0].TIR   = (CAN_tx_msg->id) << 21;
    
    CAN1->sTxMailBox[0].TDTR &= ~(0xF);
    CAN1->sTxMailBox[0].TDTR |= CAN_tx_msg->len & 0xFUL;
    
    CAN1->sTxMailBox[0].TDLR  = (((uint32_t) CAN_tx_msg->data[3] << 24) |
                                 ((uint32_t) CAN_tx_msg->data[2] << 16) |
                                 ((uint32_t) CAN_tx_msg->data[1] <<  8) |
                                 ((uint32_t) CAN_tx_msg->data[0]      ));
    CAN1->sTxMailBox[0].TDHR  = (((uint32_t) CAN_tx_msg->data[7] << 24) |
                                 ((uint32_t) CAN_tx_msg->data[6] << 16) |
                                 ((uint32_t) CAN_tx_msg->data[5] <<  8) |
                                 ((uint32_t) CAN_tx_msg->data[4]      ));

    CAN1->sTxMailBox[0].TIR  |= 0x1UL;
    while(CAN1->sTxMailBox[0].TIR & 0x1UL && count++ < 1000000);
     
     if (!(CAN1->sTxMailBox[0].TIR & 0x1UL)) return;
     
     //Sends error log to screen
     while (CAN1->sTxMailBox[0].TIR & 0x1UL)
     {
         Serial.println(CAN1->ESR);        
         Serial.println(CAN1->MSR);        
         Serial.println(CAN1->TSR);
        
     }
 }

void SendData()   // Send can messages in 32Hz phase from timer interrupt. This is important to make cluster work smoothly
{
  digitalWrite(pin, !digitalRead(pin)); // Just to see with internal led that CAN messages are being sent
  CAN_msg_RPM.data[2]= rpmLSB; //RPM LSB
  CAN_msg_RPM.data[3]= rpmMSB; //RPM MSB
  CANSend(&CAN_msg_RPM);
  //Send CLT and TPS
  
  CAN_msg_CLT_TPS.data[1]= CLT; //Coolant temp
  CAN_msg_CLT_TPS.data[5]= TPS; //TPS value. Don't know scaling or does this even have any effect to anyhting
  CANSend(&CAN_msg_CLT_TPS);

  // Send fuel consumption and error lights
  if (CEL < 120){  
    if (CEL < 60){
      CAN_msg_MPG_CEL.data[0]= 0x12;  // keep CEL And EML on for about 2 seconds
    }
    else{
      CAN_msg_MPG_CEL.data[0]= 0x02;  // keep CEL on for about 4 seconds
      }
    CEL++;
    }
  else{
    CAN_msg_MPG_CEL.data[0]= 0x00;  //CEL off
  }
  updatePW = updatePW + PW;
    if (updatePW > 5000){
      PWcount = PWcount+1; // fuel consumption is measured by rate of change in instrument cluster. So PW counter is increased by steps of one. And rate of update depends on PW from speeduino. This isn't yet working correctly.
      if (PWcount == 0xFFFF)
      {
        PWcount = 0;
      }
      updatePW = 0;
    }
  pwMSB = PWcount >> 8;  //split to high and low byte
  pwLSB = PWcount;
  CAN_msg_MPG_CEL.data[1]= pwLSB;  //LSB Fuel consumption
  CAN_msg_MPG_CEL.data[2]= pwLSB;  //MSB Fuel Consumption
  CAN_msg_MPG_CEL.data[3]= tempLight ;  //Overheat light
  CANSend(&CAN_msg_MPG_CEL);

}
 
void setup(){
 pinMode(pin, OUTPUT);
 Serial3.begin(115200);  // baudrate for Speeduino is 115200
 Serial.begin(115200); // for debugging

    CANInit(CAN_500KBPS); //init can at 500KBPS speed
    Serial.println("CAN BUS init!");
	
  CAN_msg_RPM.len = 8; //8 bytes in can message
  CAN_msg_CLT_TPS.len = 8;
  CAN_msg_MPG_CEL.len = 8;
  CAN_msg_RPM.id = 0x316; //CAN ID for RPM message is 0x316
  CAN_msg_CLT_TPS.id = 0x329; //CAN ID for CLT and TSP message is 0x329
  CAN_msg_MPG_CEL.id = 0x545; //CAN ID for fuel consumption and CEl light is 0x545

  // send this message to get rid of EML light and also set the static values for the message
  
  CAN_msg_MPG_CEL.data[0]= 0x02;  //error State
  CAN_msg_MPG_CEL.data[1]= 0x00;  //LSB Fuel consumption
  CAN_msg_MPG_CEL.data[2]= 0x00;  //MSB Fuel Consumption
  CAN_msg_MPG_CEL.data[3]= 0x00;  //Overheat light
  CAN_msg_MPG_CEL.data[4]= 0x7E;
  CAN_msg_MPG_CEL.data[5]= 0x10;
  CAN_msg_MPG_CEL.data[6]= 0x00;
  CAN_msg_MPG_CEL.data[7]= 0x18;
  CANSend(&CAN_msg_MPG_CEL);
  
//set the static values for the other two messages
  CAN_msg_RPM.data[0]= 0x00;
  CAN_msg_RPM.data[1]= 0x07;
  CAN_msg_RPM.data[4]= 0x65;
  CAN_msg_RPM.data[5]= 0x12;
  CAN_msg_RPM.data[6]= 0x0;
  CAN_msg_RPM.data[7]= 0x62;
  
  CAN_msg_CLT_TPS.data[0]= 0x07;
  CAN_msg_CLT_TPS.data[2]= 0xB2;
  CAN_msg_CLT_TPS.data[3]= 0x19;
  CAN_msg_CLT_TPS.data[4]= 0x0;
  CAN_msg_CLT_TPS.data[6]= 0x0;
  CAN_msg_CLT_TPS.data[7]= 0x0;
// zero the data to be sent by CAN bus, so it's not just random garbage

 CLT = 0;
 PW = 0;
 PWcount = 0;
 rpmLSB = 0;
 rpmMSB = 0;
 pwLSB = 0;
 pwMSB = 0;
 CEL = 0;
 readCLT = 20;

//setup hardwaretimer to send data for instrument cluster in 32Hz pace
#if defined(TIM1)
  TIM_TypeDef *Instance = TIM1;
#else
  TIM_TypeDef *Instance = TIM2;
#endif
  HardwareTimer *SendTimer = new HardwareTimer(Instance);
  SendTimer->setOverflow(32, HERTZ_FORMAT); // 32 Hz
  SendTimer->attachInterrupt(1, SendData);
  SendTimer->setMode(1, TIMER_OUTPUT_COMPARE);
  SendTimer->resume();
    
 requestData(); // all set. Start requesting data from speeduino
}

void requestData() {
  Serial3.write("A"); //Send A to request real time data
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

      RPM            = ((SpeedyResponse [16] << 8) | (SpeedyResponse [15])); // RPM low & high (Int) TBD: probaply no need to split high and low bytes etc. this could be all simpler
      PW            = ((SpeedyResponse [22] << 8) | (SpeedyResponse [21])); // PW low & high (Int) TBD: probaply no need to split high and low bytes etc. this could be all simpler
      TPS            = SpeedyResponse[25];
  	
    RPM = RPM * 6.4; // RPM conversion factor for e46/e39 cluster   
    rpmMSB = RPM >> 8;  //split to high and low byte
    rpmLSB = RPM;
    CLT = (SpeedyResponse[8] -40)*4/3+64;	// CLT conversion factor for e46/e39 cluster
}

void loop() {
 if (Serial3.available () > 0) {  // read bytes from serial3
   SpeedyResponse[ByteNumber ++] = Serial3.read();
 }
 if (ByteNumber > (75)){          // After 75 bytes all the data from speeduino has been received so time to process it (A + 74 databytes)
   oldtime = millis();          // All ok. zero out timeout calculation
   ByteNumber = 0;              // zero out the byte number pointer
   processData();               // do the necessary processing for received data
   displayData();               // only required for debugging
   requestData();               //restart data reading
 }
   if ( (millis()-oldtime) > 500) { // timeout if for some reason reading serial3 fails
    oldtime = millis();
    ByteNumber = 0;             // zero out the byte number pointer
    requestData();              //restart data reading
    }
}
