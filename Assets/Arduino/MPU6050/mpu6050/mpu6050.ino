// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;

void setup() {
  Serial.begin(57600);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
}

void loop() {
  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  //Mostrar las lecturas separadas por un [tab]
  //Serial.print("a[x y z] g[x y z]:\t");
  Serial.print(ax*(9.81/16384.0)); Serial.print(",");
  Serial.print(ay*(9.81/16384.0)); Serial.print(",");
  Serial.print(az*(9.81/16384.0)); Serial.print(",");
  Serial.print(gx*(250.0/32768.0)); Serial.print(",");
  Serial.print(gy*(250.0/32768.0)); Serial.print(",");
  Serial.println(gz*(250.0/32768.0));

  delay(100);
}
