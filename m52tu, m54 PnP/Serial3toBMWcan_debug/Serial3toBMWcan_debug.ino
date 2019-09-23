
#include <SPI.h>
#include <mcp_can.h>

static uint32_t oldtime=millis();

const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

int CLT;
int i;

unsigned int RPM;

byte TPS,tempLight;
byte response[100]; //storage buffer for realtime data i tried values 75 - 100 (100 works)

int rpmLSB;   //RPM Least significant byte for RPM message
int rpmMSB;  //RPM most significant byte for RPM message
int pwLSB;   //RPM Least significant byte for RPM message
int pwMSB;  //RPM most significant byte for RPM message

uint8_t data[8];

void setup()
{

  // SERIAL
  Serial.begin(9600);     // Serial monitor for debugging
  Serial3.begin(115200);    // Speeduino serial3
  
      while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
	
  data[0]= 0x02;  //error State
  data[1]= 0x00;  //LSB Fuel consumption
  data[2]= 0x00;  //MSB Fuel Consumption
  data[3]= 0x00;  //Overheat light
  data[4]= 0x7E;
  data[5]= 0x10;
  data[6]= 0x00;
  data[7]= 0x18;
	CAN.sendMsgBuf(0x545,0, 8, data);

 CLT = 0;
 rpmLSB = 0;
 rpmMSB = 0;
 pwLSB = 0;
 pwMSB = 0;
 
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 63036;            // preload timer 65536-16MHz/256/25Hz
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
 
}

ISR(TIMER1_OVF_vect)        // Send can messages every 25Hz from timer interrupt
{
  TCNT1 = 63036;            // preload timer


  //Send RPM
  
  data[1]= 0x07;
  data[2]= rpmLSB; //RPM LSB
  data[3]= rpmMSB; //RPM MSB
  data[4]= 0x65;
  data[5]= 0x12;
  data[6]= 0x0;
  data[7]= 0x62;
  CAN.sendMsgBuf(0x316,0, 8, data);

  //Send CLT and TPS
  
  data[0]= 0x07;
  data[1]= CLT; //Coolant temp
  data[2]= 0xB2;
  data[3]= 0x19;
  data[4]= 0x0;
  data[5]= TPS; //TPS value. Don't know scaling or does this even have any effect to anyhting
  data[6]= 0x0;
  data[7]= 0x0;
  CAN.sendMsgBuf(0x329,0, 8, data);

  // Send fuel consumption and error lights
  
  data[0]= 0x00;  //error State
  data[1]= pwLSB;  //LSB Fuel consumption (needs conversion, but this is what it is for now)
  data[2]= pwLSB;  //MSB Fuel Consumption
  data[3]= tempLight ;  //Overheat light
  data[4]= 0x7E;
  data[5]= 0x10;
  data[6]= 0x00;
  data[7]= 0x18;
  CAN.sendMsgBuf(0x545,0, 8, data);
}

void loop()
{
  if ( (millis()-oldtime) > 100) {
    oldtime = millis();
    Serial3.write("A");            // sends speeduino "A" command to request realtime data
    // Speedy sends an "A" back, and we need to account for this in the buffer, so we start the loop from -1 as its the first byte.
    while (Serial3.available() == 0) {} // while the data is availabe to Serial3 do subroutine below
  
    { // SERIAL_PACKET_SIZE   93 must match ochBlockSize in ini file
      for (i = -1; i < 93; i++)   // reads 93 SERIAL_PACKET and stores it all in buffer ready to read n print
  
      {
        response[i] = Serial3.read();
      }
    }
  
    CLT            = response[7];   // Int
    RPM            = ((response [15] << 8) | (response [14])); // RPM low & high (Int)
    pwLSB          = response[21];
    pwMSB          = response[22];
    TPS            = response[24];  // Byte

      Serial.print ("RPM-"); Serial.print (RPM); Serial.print("\t");
      Serial.print ("pwLSB-"); Serial.print (pwLSB); Serial.print("\t");
      Serial.print ("CLT-"); Serial.print (CLT); Serial.print("\t");
      Serial.print ("TPS-"); Serial.print (TPS); Serial.println("\t");
  
    RPM = RPM * 6.4; // RPM conversion factor for e46/e39 cluster
    rpmMSB = RPM >> 8;	//split to high and low byte
    rpmLSB = RPM;
    CLT = (CLT-40) * (4/3) + 64;	// CLT conversion factor for e46/e39 cluster
  		
    if(CLT>229){ // overheat light on if value is 229 or higher
      tempLight = 8;  // hex 08 = Overheat light on
    }
    else {
      tempLight = 0; // hex 00 = overheat light off
    }
  

  }
}  
