


byte SpeedyResponse[76]; //The data buffer for the serial3 data
int ByteNumber;
int rpmLSB;   //RPM Least significant byte for RPM message
int rpmMSB;  //RPM most significant byte for RPM message
int pwLSB;   //RPM Least significant byte for RPM message
int pwMSB;  //RPM most significant byte for RPM message
int CLT;
unsigned int RPM;

byte TPS,tempLight;

void setup(){
 Serial3.begin(115200);  // baudrate for Speeduino is 115200
 Serial.begin(9600); // for debugging
 requestData();
}

//Send A to request data from Speeduio
void requestData() {
  Serial3.write("A");
}

//display the values I need in serial monitor for debugging
void displayData(){
      Serial.print ("RPM-"); Serial.print (RPM); Serial.print("\t");
      Serial.print ("pwLSB-"); Serial.print (pwLSB); Serial.print("\t");
      Serial.print ("CLT-"); Serial.print (CLT); Serial.print("\t");
      Serial.print ("TPS-"); Serial.print (TPS); Serial.println("\t");
}

void processData(){
    CLT            = SpeedyResponse[8];   // Int
    RPM            = ((SpeedyResponse [16] << 8) | (SpeedyResponse [15])); // RPM low & high (Int)
    pwLSB          = SpeedyResponse[22];
    pwMSB          = SpeedyResponse[23];
    TPS            = SpeedyResponse[25];  // Byte
}

void loop() {
 if (Serial3.available () > 0) {
   SpeedyResponse[ByteNumber ++] = Serial3.read();
 }
 if (ByteNumber > 75){// process it
   ByteNumber = 0;  // ready for next time
   processData();
   displayData();
   requestData();  //restart data
 }
}
