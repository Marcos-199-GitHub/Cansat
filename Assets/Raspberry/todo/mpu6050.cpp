//Libreria con los metodos del i2c
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <iostream>

#include "./arduino_pi_wrapper.h"
/////////Funciones del MPU///////////////////////
//Direccion del MPU: b110100XY (el bit X depende del estado logico del pin AD0, el bit Y depende del modo: 0 - escritura al sensor, 1 - lectura del sensor)
#define MPUADDRESS 0x68  //B1101000 //0xD0

//Registros
#define STARTADDRESS 59
#define GYRO_CONFIG (0x1B)   // Gyroscope Configuration
#define ACCEL_CONFIG (0x1C)  // Accelerometer Configuration
#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B  // Power Management 1
#define ACCEL_XOUT_H (0x3B)
#define GYRO_XOUT_H (0x43)
#define TEMP_OUT_H (0x41)
#define INT_ENABLE 56 //Interrupt enable
#define INT_STATUS 58 //Interrupt status

#define ACCEL_MAX_FACTOR 16384
#define GYRO_MAX_FACTOR 131

using namespace std;


_Serial Serial("/dev/serial0");
int serialFd;
int mpuFd;
//Factor por el que se divide la aceleracion para convertirla a unidades estandar
//Depende de la configuracion del MPU
int accelFactor = 0;
//Lo mismo que el factor de aceleracion, pero para el giroscopio
float gyroFactor = 0;

struct Vector {
  float X;
  float Y;
  float Z;
};
struct VectorInt{
  int16_t X;
  int16_t Y;
  int16_t Z;
};



uint8_t fastRegister8(uint8_t reg);
uint8_t readRegister8(uint8_t reg);
void writeRegister8(uint8_t reg, uint8_t value);
void writeRegisterBit(uint8_t reg, uint8_t pos, bool state);

uint8_t fastRegister8(uint8_t reg)
{
    uint8_t value;
    value = wiringPiI2CReadReg8 (mpuFd, reg) ;
    return value;
}
// Read 8-bit from register
uint8_t readRegister8(uint8_t reg)
{
    return fastRegister8(reg);
}
// Write 8-bit to register
void writeRegister8(uint8_t reg, uint8_t value)
{
    wiringPiI2CWriteReg8 (mpuFd, reg, value) ;
}
void writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = readRegister8(reg);

    if (state)
    {
        value |= (1 << pos);
    } else 
    {
        value &= ~(1 << pos);
    }

    writeRegister8(reg, value);
}


void accelConfig(uint8_t mode){
  uint8_t value = readRegister8(ACCEL_CONFIG);
  //uint8_t mode = (value >> 3 ) & 0b00000011;
  uint8_t nValue = (mode << 3) | value;
  //Solo modificar los bits 4 y 5
  writeRegister8(ACCEL_CONFIG,nValue);
  //Recordar que el operador tambien sirve para dividir / 2
  accelFactor = ACCEL_MAX_FACTOR >> mode;
  /*
  switch (mode){
    case 0:
      //+-2G
      accelFactor = 16384;
    break;
    case 1:
      //+-4G
      accelFactor = 8192;
    break;
    case 3:
      //+-8G
      accelFactor = 4096;
    break;
    case 4:
      //+-16G
      accelFactor = 2048;
    break;
  }
  */
}
void gyroConfig(uint8_t mode){
  uint8_t value = readRegister8(GYRO_CONFIG);
  //uint8_t mode = (value >> 3 ) & 0b00000011;
  uint8_t nValue = (mode << 3) | value;
  //Solo modificar los bits 4 y 5
  writeRegister8(GYRO_CONFIG,nValue);
  gyroFactor = GYRO_MAX_FACTOR / (pow(2,mode));
  /*
  switch (mode){
    case 0:
      //+-250 °/s
      gyroFactor = 131;
    break;
    case 1:
      //+-4G
      accelFactor = 8192;
    break;
    case 3:
      //+-8G
      accelFactor = 4096;
    break;
    case 4:
      //+-16G
      accelFactor = 2048;
    break;
  }*/
}

/////////////////////////////////////////////////

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


VectorInt ra;
Vector pr;
VectorInt accOffsets;
VectorInt gyroOffsets;
int uncertaintyBits;




void setClockSource(int source) {
  uint8_t value;
  value = readRegister8(PWR_MGMT_1);
  value &= 0b11111000;
  value |= source;
  writeRegister8(PWR_MGMT_1, value);
}


VectorInt readAccelRaw(void){
  int i=0;
  uint8_t xha = readRegister8(ACCEL_XOUT_H+i); i++;
  uint8_t xla = readRegister8(ACCEL_XOUT_H+i); i++;
  uint8_t yha = readRegister8(ACCEL_XOUT_H+i); i++;
  uint8_t yla = readRegister8(ACCEL_XOUT_H+i); i++;
  uint8_t zha = readRegister8(ACCEL_XOUT_H+i); i++;
  uint8_t zla = readRegister8(ACCEL_XOUT_H+i); i++;

  ra.X = xha << 8 | xla;
  ra.Y = yha << 8 | yla;
  ra.Z = zha << 8 | zla;

  ra.X = ra.X & (~uncertaintyBits);
  ra.Y = ra.Y & (~uncertaintyBits);
  ra.Z = ra.Z & (~uncertaintyBits);

  ra.X -= accOffsets.X;
  ra.Y -= accOffsets.Y;
  ra.Z -= accOffsets.Z;
  return ra;

}
Vector readAccel(void){
  ra = readAccelRaw();
  //Dividir sobre el factor para convertir a m/s2
  pr.X = (float)ra.X/accelFactor;
  pr.Y = (float)ra.Y/accelFactor;
  pr.Z = (float)ra.Z/accelFactor;
  return pr;

}
VectorInt readGyroRaw(void){
  int i=0;
  uint8_t xha = readRegister8(GYRO_XOUT_H+i); i++;
  uint8_t xla = readRegister8(GYRO_XOUT_H+i); i++;
  uint8_t yha = readRegister8(GYRO_XOUT_H+i); i++;
  uint8_t yla = readRegister8(GYRO_XOUT_H+i); i++;
  uint8_t zha = readRegister8(GYRO_XOUT_H+i); i++;
  uint8_t zla = readRegister8(GYRO_XOUT_H+i); i++;

  ra.X = xha << 8 | xla;
  ra.Y = yha << 8 | yla;
  ra.Z = zha << 8 | zla;
  ra.X = ra.X & (~uncertaintyBits);
  ra.Y = ra.Y & (~uncertaintyBits);
  ra.Z = ra.Z & (~uncertaintyBits);

  ra.X -= gyroOffsets.X;
  ra.Y -= gyroOffsets.Y;
  ra.Z -= gyroOffsets.Z;
  return ra;
}
Vector readGyro(void){
  ra = readGyroRaw();
  //Dividir sobre el factor para convertir a °/s
  pr.X = (float)ra.X/gyroFactor;
  pr.Y = (float)ra.Y/gyroFactor;
  pr.Z = (float)ra.Z/gyroFactor;
  return pr;
}
int16_t readTempRaw(void){
  int i=0;
  uint8_t th = readRegister8(TEMP_OUT_H+i); i++;
  uint8_t tl = readRegister8(TEMP_OUT_H+i); i++;

  int16_t T = th<<8 | tl;
  return T;
}
float readTemp(void){
  int16_t T = readTempRaw();
  float TempC = 36.53 + (float)T/340;
  return TempC;
}

void MPUSetup(void){
  //debe devolver la direccion del MPU, sino, el dispositivo no es el correcto
  uint8_t f = fastRegister8(WHO_AM_I);
  //Serial.println(f);
  while(1){
  if (f != 0x68) {
    Serial.println("Incorrect Device");
    }
  else {
Serial.println("Correct MPU");
break;}
  delay(1000);
}
  //Configura el reloj como un reloj interno dependiente del eje x del acelerometro
  setClockSource(0b001);
  //configurar el rango de medicion del acelerometro: +-2G (0), +-4G (1), +-8G (2), +-16G (3)
  //Esto modificando el registro 28 - ACCEL_CONFIG, que tambien se utiliza para auto test del acelerometro
  //La instruccion final sera 000[2 bits de la precision]000; como la sonda no se va a poner a aceleraciones mucho mayores a 1G,
  //+-2G es la mejor opcion, es decir: 00000000
  accelConfig(0);

  //Igual se debe configurar el giroscopio, los rangos son: +-250°/s (0), +-500°/s (1), +-1000°/s (2), +-2000°/s (3)
  //Instruccion: 000[rango]000, se elige +-2000 (11), 00011000
  //+- 250 °/s (0), +-500°/s (1), +- 1000 °/s (2), +-2000°/s (3)
  gyroConfig(0);
  //Activar las interrupciones de datos, que detectan cuando hay datos nuevos en el sensor
  writeRegisterBit(INT_ENABLE,0,1);

  //Enciende el MPU (pone el modo de SLEEP como desactivado)
  writeRegisterBit(PWR_MGMT_1, 6, 0);
Serial.println("MPU configuration done");

}
bool dataReady(){
  uint8_t f = readRegister8(INT_STATUS);
  //Solo interesa el primer bit
  f = f & 1;
  return f;
}

//En el protocolo i2c hay dispositivos maestros y esclavos
//Los maestros eligen de que direccion leer o escribir (al parecer, todos los dispositivos tiene una direccion unica)
void setup() {
  //Serial = _Serial::_Serial("/dev/tty0");
  Serial.debug = 1;
  serialFd = Serial.begin(9600);
  //para inicializar el dispositivo como maestro, en caso de que el dispositivo
  // tenga mas de un i2c, se indica en el parametro
  mpuFd = wiringPiI2CSetup (MPUADDRESS) ;
  if (mpuFd == -1){
    cout << "Error inicializando el i2c" << endl;
  }
  if (serialFd == -1){
    cout << "Error inicializando el puerto serial" << endl;
  }
  MPUSetup();
  accOffsets = {0};
  gyroOffsets = {0};
  uncertaintyBits = 0b001;
  delay(1000);
Serial.println("Setup done");
  // accOffsets.X = -1657;
  // accOffsets.Y 0= -1077;
  // accOffsets.Z = 1516;

  // gyroOffsets.X = 80;
  // gyroOffsets.Y = 34;
  // gyroOffsets.Z = 45;
  
}


void loop() {
  //Cada dato de aceleracion [X, Y, Z] consta de 2 bytes (16 bits) en formato de "Complemento a 2" (basicamente un signed int)
  //float Acc[] = {0.0,0.0,0.0}; //X,Y,Z
  //Vector Acc = readRawAccel();
/*
  while (!Serial.available()){

  }
*/
  if (!dataReady()){
    return;
  }
  Vector Acc = readAccel();
  Vector Gyro = readGyro();
  float temp = readTemp();
  //+-2G = 16384 per byte (0 - 65535 per 16 bits)
  // AccG.Z = mapDouble(Acc.Z, -32768, 32767, -2, 2);
  // AccG.X = mapDouble(Acc.X, -32768, 32767, -2, 2);
  // AccG.Y = mapDouble(Acc.Y, -32768, 32767, -2, 2);
  Serial.println("^");
  Serial.print("A: ");
  Serial.println(accelFactor);
  Serial.print("Z: ");
  Serial.println(Acc.Z,5);
  Serial.print("X: ");
  Serial.println(Acc.X,5);
  Serial.print("Y: ");
  Serial.println(Acc.Y,5);

  Serial.print("W: ");
  Serial.println(gyroFactor);
  Serial.print("Wz: ");
  Serial.println(Gyro.Z,5);
  Serial.print("Wx: ");
  Serial.println(Gyro.X,5);
  Serial.print("Wy: ");
  Serial.println(Gyro.Y,5);
  Serial.println("$");
/*
  Serial.print("C: ");
  Serial.println(temp, 5);
*/
  //delay(1000);
}
int main(){
setup();
while(1){loop();}
}
