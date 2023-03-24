import time
BMM150_ADDRESS = 0x13
_REG_POWER_CONTROL = 0x4B
BMM150_START_UP_TIME = 0.003
class BMM150:

    def __init__(self,i2c) -> None:
        self.device = i2c
        self.id = 0
        #Enciende el dispositivo
        self.writeRegister8(_REG_POWER_CONTROL,0b10000011)
        time.sleep(BMM150_START_UP_TIME)



    def writeRegister8(self,reg,dato):
        self.device.writeto(BMM150_ADDRESS,bytes([reg]),stop=False)
        self.device.writeto(BMM150_ADDRESS,bytes([dato]),stop=True)
        
    def checkId(self) -> bool:
        data = bytearray(1)
        self.device.writeto(BMM150_ADDRESS,bytes([0x40]),stop=False)
        self.device.readfrom_into(BMM150_ADDRESS,data)
        self.id = data[0]
        return data[0]==0x32
