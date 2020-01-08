// This code is meant to read real time data from Speeduino EFI using serial3 connection in speeduino and convert that to CAN messages for BMW e39/e46 instrument clusters
// The hardware that the code is meant to be used is the built in CAN-bus interface in BMW m52tu/m54 speeduino PnP PCB that consists ATmega 328p processor and MCP2515+MCP2551 CAN bus chips. https://github.com/pazi88/Speeduino-M5x-PCBs/tree/master/m52tu%2C%20m54%20PnP
// Created by pazi88 and there is no guarantee at all that any of this will work. Use Arduino Nano as board in Arduino IDE

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.

#include <SPI.h>
#include "mcp_can.h"

const int SPI_CS_PIN = 10;   // pin10 is wired as CS pin in the PCB
MCP_CAN CAN(SPI_CS_PIN);     // Set CS pin in CAN library

static uint32_t oldtime=millis();   // for the timeout
byte SpeedyResponse[100]; //The data buffer for the serial3 data
byte ByteNumber;  // pointer to which byte number we are reading currently
byte rpmLSB;   //RPM Least significant byte for RPM message
byte rpmMSB;  //RPM most significant byte for RPM message
byte pwLSB;   //RPM Least significant byte for RPM message
byte pwMSB;  //RPM most significant byte for RPM message
byte CEL;   //timer for how long CEL light be kept on
byte TPS;  //TPS value
byte tempLight; // overheat light on/off
int CLT;   // to store coolant temp
unsigned int RPM;   //RPM from speeduino

uint8_t data[8];    // data that will be sent to CAN bus

void setup(){
 digitalWrite( 0, HIGH );
 digitalWrite( 1, HIGH );
 Serial.begin(115200);  // baudrate for Speeduino is 115200
      while (CAN_OK != CAN.begin(CAN_500KBPS)){}          // init can bus : baudrate = 500k

  // send this message to get rid of EML light
  
  data[0]= 0x02;  //error State
  data[1]= 0x00;  //LSB Fuel consumption
  data[2]= 0x00;  //MSB Fuel Consumption
  data[3]= 0x00;  //Overheat light
  data[4]= 0x7E;
  data[5]= 0x10;
  data[6]= 0x00;
  data[7]= 0x18;
	CAN.sendMsgBuf(0x545,0, 8, data);

// zero the data to be sent by CAN bus, so it's not just random garbage

 CLT = 0;
 rpmLSB = 0;
 rpmMSB = 0;
 pwLSB = 0;
 pwMSB = 0;
 CEL = 0;
 
  // initialize timer1 to send CAN data in 32Hz rate (about)
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 63600;            // preload timer 
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << TOIE1);
  interrupts();

 requestData(); // all set. Start requesting data from speeduino
}

//Send A to request data from Speeduio
void requestData() {
  Serial.write("A");
}

void processData(){   // necessary conversion for the data before sending to CAN BUS

    RPM            = ((SpeedyResponse [16] << 8) | (SpeedyResponse [15])); // RPM low & high (Int) TBD: probaply no need to split high and low bytes etc. this could be all simpler
    pwLSB          = SpeedyResponse[22];
    pwMSB          = SpeedyResponse[23];
    TPS            = SpeedyResponse[25];  // Byte
	
    RPM = RPM * 6.4; // RPM conversion factor for e46/e39 cluster
    rpmMSB = RPM >> 8;	//split to high and low byte
    rpmLSB = RPM;
    CLT = (SpeedyResponse[8])*1.3+14;	// CLT conversion factor for e46/e39 cluster
    // actual calculation is (SpeedyResponse[8] -40)*4/3+64, but upper one has almost same result faster
  		
    if(CLT>229){ // overheat light on if value is 229 or higher
      tempLight = 8;  // hex 08 = Overheat light on
    }
    else {
      tempLight = 0; // hex 00 = overheat light off
    }
}

ISR(TIMER1_OVF_vect)        // Send can messages on timer interrupt
{
  TCNT1 = 63600;            // preload timer
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
  if (CEL < 60){	// keep CEL on for about 2 seconds
    data[0]= 0x02;  //CEL on
    CEL++;
  }
  else{
    data[0]= 0x00;  //CEL off
  }
  data[1]= pwLSB;  //LSB Fuel consumption (needs conversion, but this is what it is for now)
  data[2]= pwLSB;  //MSB Fuel Consumption
  data[3]= tempLight ;  //Overheat light
  data[4]= 0x7E;
  data[5]= 0x10;
  data[6]= 0x00;
  data[7]= 0x18;
  CAN.sendMsgBuf(0x545,0, 8, data);
}

void loop() {
 if (Serial.available () > 0) {  // read bytes from serial
   SpeedyResponse[ByteNumber ++] = Serial.read();
 }
 if (ByteNumber > 75){          // After 75 bytes all the data from speeduino has been received so time to process it (A + 74 databytes)
   oldtime = millis();          // All ok. zero out timeout calculation
   ByteNumber = 0;              // zero out the byte number pointer
   processData();               // do the necessary processing for received data
   delay(200); 
   requestData();               //restart data reading
 }
   if ( (millis()-oldtime) > 100) { // timeout if for some reason reading serial fails
    oldtime = millis();
    ByteNumber = 0;             // zero out the byte number pointer
    requestData();              //restart data reading
    }
}
