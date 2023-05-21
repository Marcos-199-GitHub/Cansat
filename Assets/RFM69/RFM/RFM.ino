 #include <SPI.h>
#include "rfm.h"


//Pin 10: SS
//Pin 11: MOSI
//Pin 12: MISO
//Pin 13: SCK

const int slaveSelectPin = 10;
 
void setup() {
  // poner el pin SS como salida y en estado alto
  pinMode(slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, HIGH);
  pinMode(IRQ, INPUT);
  // Inicializar bus SPI
  SPI.begin();
  Serial.begin(9600);
  RFMinit();
  //Serial.println("Starting");
  RFM69_Config();
  Delay(100);
  RFM69_EntryTx();
}
 
void loop() {
char* s = receive();

//Serial.println(s);
//char dat = readFIFO(); 

char res = rfmSend("hola",4);
/*
//readAllRegs();
//Serial.println(dat,HEX);
//if (res)continue;
  //Serial.println("Enviado");
  */
  /*
  RFM69_RxPacket();
  //char res = rfmSend("hola",4);
  println(receive());
  char x = Serial.read();
    if (x=='a'){
    readFIFO();
    }
  if (x=='r'){
    readAllRegs();
    }
*/
 //Serial.println("Nothing");
delay(100);
}
 
