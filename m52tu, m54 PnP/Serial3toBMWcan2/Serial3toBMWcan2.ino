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

static uint32_t oldtime=millis();   // for the timeout
byte SpeedyResponse[16]; //The data buffer for the serial3 data
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
 Serial.begin(115200);  // baudrate for Speeduino is 115200
 digitalWrite( 0, HIGH );
 digitalWrite( 1, HIGH );
 requestData(); // all set. Start requesting data from speeduino
}

//Send A to request data from Speeduio
void requestData() {
  Serial.write("r"); //new type real time data
  Serial.write(0x00);	//Speeduino TS canID, default: 0x201
  Serial.write(0x30);	//commad type
  Serial.write(15); //offset. RPM is at 16
  Serial.write(0x00);
  Serial.write(10);	//how mony bytes we need back. TPS is 25 so 25-16 = 9
  Serial.write(0x00);
}

void displayData(){
       Serial.print (SpeedyResponse[2]);
       Serial.print (SpeedyResponse[3]); 
       Serial.print (SpeedyResponse[4]); 
       Serial.print (SpeedyResponse[5]); 
       Serial.print (SpeedyResponse[6]); 
       Serial.print (SpeedyResponse[7]); 
       Serial.print (SpeedyResponse[8]); 
       Serial.print (SpeedyResponse[9]); 
       Serial.print (SpeedyResponse[10]); 
}


void loop() {
 if (Serial.available () > 0) {  // read bytes from serial
   SpeedyResponse[ByteNumber ++] = Serial.read();
 }
 if (ByteNumber > 12){          // After 75 bytes all the data from speeduino has been received so time to process it (A + 74 databytes)
   oldtime = millis();          // All ok. zero out timeout calculation
   ByteNumber = 0;              // zero out the byte number pointer
   requestData();               //restart data reading
 }
   if ( (millis()-oldtime) > 100) { // timeout if for some reason reading serial fails
    oldtime = millis();
    ByteNumber = 0;             // zero out the byte number pointer
    requestData();              //restart data reading
    }
}
