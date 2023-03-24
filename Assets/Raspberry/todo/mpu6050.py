import time
import board


#/////////Funciones del MPU///////////////////////
#//Direccion del MPU: b110100XY (el bit X depende del estado logico del pin AD0, el bit Y depende del modo: 0 - escritura al sensor, 1 - lectura del sensor)
MPUADDRESS = 0x68  #B1101000 //0xD0

#Registros
STARTADDRESS = 59
GYRO_CONFIG = (0x1B)   #Gyroscope Configuration
ACCEL_CONFIG = (0x1C)  #// Accelerometer Configuration
WHO_AM_I = 0x75
PWR_MGMT_1 = 0x6B  #Power Management 1
ACCEL_XOUT_H = (0x3B)
GYRO_XOUT_H = (0x43)
TEMP_OUT_H = (0x41)
INT_ENABLE = 56 #Interrupt enable
INT_STATUS = 58 #//Interrupt statu
ACCEL_MAX_FACTOR = 16384
GYRO_MAX_FACTOR = 131




class MPU():
    """ 
void MPUSetup(void){
  //debe devolver la direccion del MPU, sino, el dispositivo no es el correcto
  uint8_t f = fastRegister8(WHO_AM_I);
  //Serial.println(f);
  if (f != 0x68) {
    Serial.println("Incorrect Device");
    while(1);
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
}
    """
    
    def __init__(self,i2c) -> None:
        self.device = i2c
        self.id = 0
        self.accelFactor = 0
        self.gyroFactor = 0
        self.rawAccel = [0,0,0]
        self.accel = [0,0,0]
        self.rawGyro = [0,0,0]
        self.gyro = [0,0,0]
        self.rawTemp = 0
        self.tempC = 0
        if (not self.checkId):
            print("Incorrect MPU")  
        time.sleep(1)
        self.setClockSource(0b001)
        self.accelConfig(0)
        self.gyroConfig(0)
        #Activar las interrupciones de datos, que detectan cuando hay datos nuevos en el sensor
        self.writeRegisterBit(INT_ENABLE,0,1);
        #Enciende el MPU (pone el modo de SLEEP como desactivado)
        self.writeRegisterBit(PWR_MGMT_1, 6, 0);


        self.accOffsets = [0,0,0];
        self.gyroOffsets = [0,0,0];
        self.uncertaintyBits = 0b1111;
        #print (self.dataReady())
        pwr = self.readRegister8(PWR_MGMT_1)
        #print(f"{hex(pwr)}")
    def update(self):
        if (self.dataReady() == 0):
            print("Data not ready")
            return
        self.readAccel()
        self.readGyro()
        self.readTemp()
    def readRegister8(self, register: int) -> int:
        """Read a byte register value and return it"""
        data = bytearray(1)
        self.device.writeto(MPUADDRESS,bytes([register]),stop=False)
        self.device.readfrom_into(MPUADDRESS,data)
        return data[0]
    def writeRegister8(self,reg,dato):
        self.device.writeto(MPUADDRESS,bytes([reg,dato]),stop=True)
        #self.device.writeto(MPUADDRESS,bytes([dato]),stop=True)
    def writeRegisterBit(self,reg,pos,state):
        value = self.readRegister8(reg)
        if (state == 1):
            value |= (1 << pos);
        else:
            value &= ~(1 << pos);
        self.writeRegister8(reg,value)
    def dataReady(self):
        f = self.readRegister8(INT_STATUS);
        #Solo interesa el primer bit
        f = f & 1
        return f;
    def checkId(self) -> bool:
        data = self.readRegister8(WHO_AM_I)
        self.id = data
        return data==MPUADDRESS
    def setClockSource(self, source):
        value = self.readRegister8(PWR_MGMT_1)
        value &= 0b11111000;
        value |= source;
        self.writeRegister8(PWR_MGMT_1, value)
    def accelConfig(self,mode):
        value = self.readRegister8(ACCEL_CONFIG)
        nValue = (mode << 3) | value;
        #Solo modificar los bits 4 y 5
        self.writeRegister8(ACCEL_CONFIG,nValue)
        #Recordar que el operador tambien sirve para dividir / 2
        self.accelFactor = ACCEL_MAX_FACTOR >> mode;
    def gyroConfig(self,mode):
        value = self.readRegister8(GYRO_CONFIG)
        #mode = (value >> 3 ) & 0b00000011;
        nValue = (mode << 3) | value;
        #Solo modificar los bits 4 y 5
        self.writeRegister8(GYRO_CONFIG,nValue)
        self.gyroFactor = GYRO_MAX_FACTOR / (pow(2,mode));
    """
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
    """
    def twoComplement(self,val: int):
        binary = bin(val)
        #Primero cortar el 0b
        binary = binary[2:]
        neg = 2*int(binary,2) - int(binary[0]+binary,2)
        return neg

    def readAccelRaw(self):
        i=0
        xha = self.readRegister8(ACCEL_XOUT_H+i)
        i+=1
        xla = self.readRegister8(ACCEL_XOUT_H+i)
        i+=1
        yha = self.readRegister8(ACCEL_XOUT_H+i)
        i+=1
        yla = self.readRegister8(ACCEL_XOUT_H+i)
        i+=1
        zha = self.readRegister8(ACCEL_XOUT_H+i)
        i+=1
        zla = self.readRegister8(ACCEL_XOUT_H+i)
        self.rawAccel[0] = xha << 8 | xla
        self.rawAccel[1] = yha << 8 | yla
        self.rawAccel[2] = zha << 8 | zla

        self.rawAccel[0] = self.rawAccel[0] & ~self.uncertaintyBits
        self.rawAccel[1] = self.rawAccel[1] & ~self.uncertaintyBits
        self.rawAccel[2] = self.rawAccel[2] & ~self.uncertaintyBits

        #Como estan en formato 2's complement, para aceptar valores negativos, hay que hacer la conversion adecuada
        self.rawAccel[0] = self.twoComplement(self.rawAccel[0])
        self.rawAccel[1] = self.twoComplement(self.rawAccel[1])
        self.rawAccel[2] = self.twoComplement(self.rawAccel[2])

        self.rawAccel[0] -= self.accOffsets[0]
        self.rawAccel[1] -= self.accOffsets[1]
        self.rawAccel[2] -= self.accOffsets[2]
        return self.rawAccel
    
    def readGyroRaw(self):
        i=0
        xha = self.readRegister8(GYRO_XOUT_H+i)
        i+=1
        xla = self.readRegister8(GYRO_XOUT_H+i)
        i+=1
        yha = self.readRegister8(GYRO_XOUT_H+i)
        i+=1
        yla = self.readRegister8(GYRO_XOUT_H+i)
        i+=1
        zha = self.readRegister8(GYRO_XOUT_H+i)
        i+=1
        zla = self.readRegister8(GYRO_XOUT_H+i)
        self.rawGyro[0] = xha << 8 | xla
        self.rawGyro[1] = yha << 8 | yla
        self.rawGyro[2] = zha << 8 | zla

        self.rawGyro[0] = self.rawGyro[0] & ~self.uncertaintyBits
        self.rawGyro[1] = self.rawGyro[1] & ~self.uncertaintyBits
        self.rawGyro[2] = self.rawGyro[2] & ~self.uncertaintyBits
        #Como estan en formato 2's complement, para aceptar valores negativos, hay que hacer la conversion adecuada
        self.rawGyro[0] = self.twoComplement(self.rawGyro[0])
        self.rawGyro[1] = self.twoComplement(self.rawGyro[1])
        self.rawGyro[2] = self.twoComplement(self.rawGyro[2])

        self.rawGyro[0] -= self.accOffsets[0]
        self.rawGyro[1] -= self.accOffsets[1]
        self.rawGyro[2] -= self.accOffsets[2]
        return self.rawGyro
    
    def readTempRaw(self):
        i=0;
        th = self.readRegister8(TEMP_OUT_H+i)
        i+=1;
        tl = self.readRegister8(TEMP_OUT_H+i)
        self.rawTemp = th<<8 | tl;
        return self.rawTemp
    def readAccel(self):
        self.readAccelRaw();
        #Dividir sobre el factor para convertir a m/s2
        self.accel[0] = self.rawAccel[0]/self.accelFactor
        self.accel[1] = self.rawAccel[1]/self.accelFactor
        self.accel[2] = self.rawAccel[2]/self.accelFactor
        return self.accel
    def readGyro(self):
        self.readGyroRaw();
        #Dividir sobre el factor para convertir a m/s2
        self.gyro[0] = self.rawGyro[0]/self.gyroFactor
        self.gyro[1] = self.rawGyro[1]/self.gyroFactor
        self.gyro[2] = self.rawGyro[2]/self.gyroFactor
        return self.gyro
    def readTemp(self):
        self.readTempRaw()
        self.tempC = 36.53 + self.rawTemp/340;
        

               
        
