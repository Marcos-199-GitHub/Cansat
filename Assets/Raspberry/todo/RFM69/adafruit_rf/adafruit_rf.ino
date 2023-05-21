
#define SSPin 10
#define SPIBAUD 100000
#define FREQ_433
#define HEX 16
#define BIN 2
//#define FREQ_433
//#define FREQ_433


#include <SPI.h>
#include<stdint.h>

#include "adafruit_rfm69_registers.h"
#include "utils.h"
#include "afadruit_rfm69.h"



int ResetPin = 9;
//PRIMER BYTE es el tamaño del array 
uint8_t synch[] = {3,0xAA,0x2D,0xD4};
//uint8_t sync_size = 3;
RFM69 radio;



void setup() {
  Serial.begin(9600);
  SPI.begin();
  pinMode(SSPin,OUTPUT);
  pinMode(ResetPin,OUTPUT);
  digitalWrite(SSPin,HIGH);
  digitalWrite(ResetPin,LOW);
  radio.init(synch,ResetPin);
  //radio.readAllRegs();
  //while (1);
}



void loop() {
  Serial.println("Ciclo");
  //radio.send("hola",4);

  int i=0;
  char* packet = radio.receive();
  if (!(packet == NULL || packet[0] == 0)){
    for (i=0;i<packet[0];i++){
  Serial.print(packet[i+1]);
  }
  Serial.println("");
  }
  //radio.readAllRegs();
  free(packet);
  delay(100);
}
