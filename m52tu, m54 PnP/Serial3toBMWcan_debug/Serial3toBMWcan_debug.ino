//Based on Garcia's code at https://speeduino.com/forum/viewtopic.php?f=19&t=2060

#include <SPI.h>
#include <mcp_can.h>

volatile int mainLoopCount;

const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

int CLT;
int i;

float PW1raw, PW1;
//byte DWELLraw, DWELLLB, DWELLHB;
unsigned int RPM;

long longRPM, MAPlong; //Has to be a long for PID calcs (Boost control)

byte TPS,tempLight;
byte response[100]; //storage buffer for realtime data i tried values 75 - 100 (100 works)

int rpmLSB;   //RPM Least significant pit from RPM calc
int rpmMSB;  //RPM most significant pit from RPM calc

uint8_t data[8];

void setup()
{

  // SERIAL
  Serial.begin(9600);     // Serial monitor for debugging
  Serial3.begin(115200);    // Speedy Rx3 /Tx3 to SlaveMega2560 Tx3 - Rx3
  
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
}
void loop()
{
 mainLoopCount++;
  if ((mainLoopCount & 63) == 1)
  {
  Serial3.write("A");            // sends speeduino "A" command to request realtime data
  // Speedy sends an "A" back, and we need to account for this in the buffer, so we start the loop from -1 as its the first byte.
  while (Serial3.available() > 0) // while the data is availabe to Serial3 do subroutine below

  { // SERIAL_PACKET_SIZE   93 must match ochBlockSize in ini file
    for (i = -1; i < 93; i++)   // reads 93 SERIAL_PACKET and stores it all in buffer ready to read n print

    {
      response[i] = Serial3.read();
    }
  }

  //  each SERIAL_PACKET data gets a name to match it data number
  //  you find all this info in speeduino.ini file
  //  you pick what data you need. so not all 89 will be used
  //  example: am not intrested in nos or canbus input so dont bother to  give those a name so not available to print.
  //  canin_gauge0     =  response [41]; // relates to canbus input SERIAL_PACKET 41
  //  or
  //  nitrousOn        =  response [81];  // nitrous ON status  SERIAL_PACKET 81



//  SECL           = response[0];   // Volatile BYTE
//  ENGINE         = response[2];   // Byte (Bit Field)
//  SYNCloss       = response[3];   // Int
//  MAP            = ((response [5] << 8) | (response [4])); // MAP low & high (Int)
//  IAT            = response[6];   // Int
  CLT            = response[7];   // Int
//  BATc           = response [8];  // Byte
//  BATv           = response[9];   // Byte
//  EGO1           = response[10];  // Byte
//  EGO1c          = response[11];  // Byte
//  IATc           = response[12];  // Byte
//  WUE            = response[13];  // Byte
  RPM            = ((response [15] << 8) | (response [14])); // RPM low & high (Int)
//  GAMMAEt        = response[17];  // Byte
//  VE             = response[18];  // Byte
//  EGO1t          = response[19];  // Byte
  PW1raw            = ((response [21] << 8) | (response [20])); // Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
//  TPSdot         = response[22];  //
//  ADV            = response[23];  // Byte
  TPS            = response[24];  // Byte
//  loopsPerSecond = ((response [26] << 8) | (response [25]));
//  freeRAM        = ((response [28] << 8) | (response [27]));
//  boostTarget    = response [29]; //
//  boostDuty      = response [30]; //
//  SPARK          = response[31];  // Byte (Bit Field)
//  RPMdot         = ((response [33] << 8) | (response [32]));
//  FLEX           = response[34]; //
//  EGO2           = response[39];  // Byte

//  DWELLraw          = ((response [90] << 8) | (response [89]));
//  DWELLLB           = response[89];
//  DWELLHB           = response[90];

  //------------- Engine Status----------------------
//  Running         = bitRead(ENGINE, 0); // Bool
//  Cranking        = bitRead(ENGINE, 1); // Bool
//  Ase             = bitRead(ENGINE, 2); // Bool
//  Warmup          = bitRead(ENGINE, 3); // Bool
  
//  tpsaccaen       = bitRead(ENGINE, 4); // Bool
//  tpsaccden       = bitRead(ENGINE, 5); // Bool
//  mapaccaen       = bitRead(ENGINE, 6); // Bool
//  mapaccden       = bitRead(ENGINE, 7); // Bool


  //------------- System Status----------------------

//  Launch_hard     = bitRead(status1, 0);
//  Launch_soft     = bitRead(status1, 1);
//  Limit_hard      = bitRead(status1, 2);
//  Limit_soft      = bitRead(status1, 3);
//  Boostcut_spark  = bitRead(status1, 4);
//  Error           = bitRead(status1, 5);
//  Idle            = bitRead(status1, 6);
//  Sync            = bitRead(status1, 7);


  /*  ******************************************   CALCS   ********************************************
      Some values have arrive as pairs of bytes. These have been joind as they arrived, but need further processing. The Serial Monitor can display what ever is sent to.
      But Nextion can only work with whole numbers. So this section will convert to correct size and turn into text those that require it.
      Convert scale, take int, take remainder, convert to text with decimal point in the middle.

  */

  PW1 = PW1raw / 1000;
  rpmMSB = RPM/40;
  CLT=CLT+140;
  
  if(CLT>229){ // lights light if value is 229 (hot)
    tempLight = 8;  // hex 08 = Overheat light on
  }
  else {
    tempLight = 0; // hex 00 = overheat light off
  }
  //----------------OUTPUT TO SERIAL MONITOR-------



    Serial.print ("RPM-"); Serial.print (RPM); Serial.print("\t");
    Serial.print ("PW-"); Serial.print (PW1); Serial.print("\t");
    Serial.print ("CLT-"); Serial.print (CLT); Serial.print("\t");
    Serial.print ("TPS-"); Serial.print (TPS); Serial.println("\t");
}  	
  //Send RPM
  
  data[1]= 0x07;
  data[2]= 0xFF;  //RPM LSB0xFF
  data[3]= rpmMSB; //RPM MSB "0x0F" = 600 RPM0x4F
  data[4]= 0x65;
  data[5]= 0x12;
  data[6]= 0x0;
  data[7]= 0x62;
	CAN.sendMsgBuf(0x316,0, 8, data);

  //Send CLT and TPS
  
  data[0]= 0x07;
  data[1]= CLT; //temp bit tdata
  data[2]= 0xB2;
  data[3]= 0x19;
  data[4]= 0x0;
  data[5]= TPS;
  data[6]= 0x0;
  data[7]= 0x0;
  CAN.sendMsgBuf(0x329,0, 8, data);

  // Send fuel consumption and error lights
  
  data[0]= 0x00;  //error State
  data[1]= PW1;  //LSB Fuel consumption
  data[2]= 0x00;  //MSB Fuel Consumption
  data[3]= tempLight ;  //Overheat light
  data[4]= 0x7E;
  data[5]= 0x10;
  data[6]= 0x00;
  data[7]= 0x18;
	CAN.sendMsgBuf(0x545,0, 8, data);

}
