// This code is meant to read real time data from Speeduino EFI using serial3 connection in speeduino and convert that to CAN messages for BMW e39/e46 instrument clusters
// The hardware that the code is meant to be used is Arduino Mega with SeedStudio CAN-bus shield including MCP2515+MCP2551 CAN bus chips.
// Created by pazi88 and there is no guarantee at all that any of this will work. Use Arduino Mega as board in Arduino IDE

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
byte SpeedyResponse[24]; //The data buffer for the serial3 data
byte ByteNumber;  // pointer to which byte number we are reading currently
byte rpmLSB;   //RPM Least significant byte for RPM message
byte rpmMSB;  //RPM most significant byte for RPM message
byte pwLSB;   //RPM Least significant byte for RPM message
byte pwMSB;  //RPM most significant byte for RPM message
byte CEL;   //timer for how long CEL light be kept on
byte readCLT; // CLT doesn't need to be updated very ofter so 
int CLT;   // to store coolant temp
byte ResponseLength; // how long response is asked from speeduino
unsigned int RPM;   //RPM from speeduino
byte TPS,tempLight;   //TPS value and overheat light on/off

uint8_t data[8];    // data that will be sent to CAN bus

void setup(){
 Serial3.begin(115200);  // baudrate for Speeduino is 115200
 Serial.begin(9600); // for debugging

 
      while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100); //TBD figure out if this can be shorter or even needed
    }
    Serial.println("CAN BUS Shield init ok!");

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
 ResponseLength = 18; //start with coolant also
 readCLT = 20;
 
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

//Send r to request data from Speeduio
void requestData() {
  // we don't need to read CLT very often, so only read once in every 20 messages
  readCLT++;
  if (readCLT >20){
      ResponseLength = 18; // to fit CLT and TPS we need 18 bytes
      readCLT = 0;  
    }
    else{
      ResponseLength = 11; // to fit RPM and TPS we need 11 bytes
    }
  
  Serial3.write("r"); //new type real time data
  Serial3.write(0x00);	//Speeduino TS canID, not used atm
  Serial3.write(0x30);	//command type, 0x30 for real time data
  Serial3.write(-ResponseLength +25); //offset for the data. 7 with CLT, 14 without
  Serial3.write(0x00); // offset is in 2 bytes. LSB first
  Serial3.write(ResponseLength);	//how mony bytes we need back.
  Serial3.write(0x00); // number of bytes is in 2 bytes. LSB first
}

//display the needed values in serial monitor for debugging
void displayData(){
      Serial.print ("RPM-"); Serial.print (RPM); Serial.print("\t");
      Serial.print ("pwMSB-"); Serial.print (pwMSB); Serial.print("\t");
      Serial.print ("CLT-"); Serial.print (CLT); Serial.print("\t");
      Serial.print ("TPS-"); Serial.print (TPS); Serial.println("\t");

}

void processData(){   // necessary conversion for the data before sending to CAN BUS
    if(SpeedyResponse[1] == 0x30){ //if there is wrong data, filter it out
      RPM            = ((SpeedyResponse [ResponseLength - 8] << 8) | (SpeedyResponse [ResponseLength - 9])); // RPM low & high (Int) TBD: probaply no need to split high and low bytes etc. this could be all simpler
      pwLSB          = SpeedyResponse[ResponseLength - 3];
      pwMSB          = SpeedyResponse[ResponseLength - 2];
      TPS            = SpeedyResponse[ResponseLength + 1];
  	
      RPM = RPM * 6.4; // RPM conversion factor for e46/e39 cluster
      rpmMSB = RPM >> 8;	//split to high and low byte
      rpmLSB = RPM;
      if (ResponseLength > 12){ //in case of CLT is read, also calculate conversion for it
        CLT = (SpeedyResponse[2])*1.3+14;	// CLT conversion factor for e46/e39 cluster
      //   actual calculation is (SpeedyResponse[8] -40)*4/3+64, but upper one has almost same result faster
      		
        if(CLT>229){ // overheat light on if value is 229 or higher
          tempLight = 8;  // hex 08 = Overheat light on
        }
        else {
          tempLight = 0; // hex 00 = overheat light off
        }
      }
    }
}

ISR(TIMER1_OVF_vect)        // Send can messages in 30Hz phase from timer interrupt. This is important to make cluster work smoothly
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
 if (Serial3.available () > 0) {  // read bytes from serial3
   SpeedyResponse[ByteNumber ++] = Serial3.read();
 }
 if (ByteNumber > (ResponseLength +1)){          // After the data from speeduino has been received so time to process it
   oldtime = millis();          // All ok. zero out timeout calculation
   ByteNumber = 0;              // zero out the byte number pointer
   processData();               // do the necessary processing for received data
   displayData();               // only required for debugging
   requestData();               //restart data reading
 }
   if ( (millis()-oldtime) > 70) { // timeout if for some reason reading serial3 fails
    oldtime = millis();
    ByteNumber = 0;             // zero out the byte number pointer
    requestData();              //restart data reading
    }
}
