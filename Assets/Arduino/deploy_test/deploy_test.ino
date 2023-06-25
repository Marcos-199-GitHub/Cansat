// Incluímos la librería para poder controlar el servo
#include <Servo.h>
#include <Wire.h>
#include <BMP280_DEV.h>

 
// Declaramos la variable para controlar el servo
Servo servoMotor;
int initialAngle = 3;
int finalAngle = 25;
float seconds = 0;
float deployTimeS = 90;
bool deployed = false;
int servoPin = 5;
int ledPin = 13;
BMP280_DEV bmp280;
float temperature,pressure,altitude;

void setup() {
  Serial.begin (9600);
  bmp280.begin(BMP280_I2C_ALT_ADDR);
  bmp280.setTimeStandby(TIME_STANDBY_2000MS);
  bmp280.startNormalConversion();



  pinMode(ledPin,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(servoPin,OUTPUT);
  digitalWrite(3,0);
  digitalWrite(4,1);


  // // Iniciamos el mon;itor serie para mostrar el resultado
  // Serial.begin(9600);
 
  // Iniciamos el servo para que empiece a trabajar con el pin 
  servoMotor.attach(servoPin);
  //servoMotor.write(finalAngle);
  //delay(5000);
  // Desplazamos a la posición 0º
  servoMotor.write(initialAngle);
  

for (int i=0;i<5;i++){
  digitalWrite(ledPin,HIGH);
  delay(500);
  digitalWrite(ledPin,LOW);
  delay(500);
  }

}
 
void loop() {

  if (!deployed){
    while (seconds < deployTimeS){
      seconds = millis()/1000;
    }
    // Esperamos 1 segundo
    delay(1000);
    digitalWrite(ledPin,HIGH);
  
    
    // Desplazamos 
    servoMotor.write(finalAngle);
    deployed = true;
  }
  else {
    //Aqui se pueden guardar datos importantes dentro de la EEPROM, aunque por ahora no hay ningun dato relevante
      if (bmp280.getMeasurements(temperature,pressure,altitude)){
    Serial.print("TEMP");
    Serial.println(temperature);
        Serial.print("PRESS");
    Serial.println(pressure);
        Serial.print("ALTITUDE");
    Serial.println(altitude);
  }

  }

}