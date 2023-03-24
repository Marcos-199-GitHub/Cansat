#include <SoftwareSerial.h>
                //rx,tx
SoftwareSerial GPS(10,11);
#define ctrlZ 0x1A
#define ctrlZButtonPin 4
void setup() {
GPS.begin(9600);
pinMode(ctrlZButtonPin,INPUT);
Serial.begin(9600); 
  //GPS.write("AT");
  Serial.write(GPS.read());
}

void loop() {
  // if(digitalRead(ctrlZButtonPin)){
  //   Serial.write(ctrlZ);
  //       GPS.write(ctrlZ);
  //   }
  if(GPS.available()>0){
    Serial.write(GPS.read());
    }  
  if(Serial.available()>0){
    GPS.write((char)Serial.read());
    }
   //delay(500);

}
