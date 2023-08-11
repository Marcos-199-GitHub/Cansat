# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_sht31d
import serial
# import digitalio # For use with SPI
import adafruit_bmp280
#import grove_bmm150
import mpu6050
#Pantalla del RFM
import adafruit_ssd1306
import math
import bmm150
# Import the RFM69 radio module.
import adafruit_rfm69
import busio
import subprocess

from digitalio import DigitalInOut, Direction, Pull
import os
import camara


#Fusion de los sensores de movimiento
import sys
sys.path.append('../../Graficas Python/Calibracion de los IMU')
import fusion
import numpy as np



DELAY = 0.001
SHT30_ADDRESS = 0x44
INIT_TIME = time.time()
PHOTO_CMD = "streamer -d /dev/video0 -s 320x240 -o "

# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)

# OR Create sensor object, communicating over the board's default SPI bus
# spi = board.SPI()
# bmp_cs = digitalio.DigitalInOut(board.D10)
# bmp280 = adafruit_bmp280.Adafruit_BMP280_SPI(spi, bmp_cs)

# change this to match the location's pressure (hPa) at sea level
bmp280.sea_level_pressure = 1013.25

# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sht30 = adafruit_sht31d.SHT31D(i2c)

print("\033[1mSensor\033[0m = SHT31-D")
print("\033[1mSerial Number\033[0m = ", sht30.serial_number, "\n")
sht30.frequency = adafruit_sht31d.FREQUENCY_1
sht30.mode = adafruit_sht31d.MODE_PERIODIC

#MPU y BMM
dev_bmm150 = bmm150.BMM150()
#print (f"BMM correct: {bmm150.checkId()}, id: {bmm150.id}")
mpu = mpu6050.MPU(i2c)
print (f"MPU correct: {mpu.checkId()}, id: {mpu.id}")

#LCD
reset_pin = DigitalInOut(board.D4)
display = adafruit_ssd1306.SSD1306_I2C(128,32,i2c,reset = reset_pin)
#Clear display
display.fill(0)
display.show()
width = display.width
height = display.height

display.text('Enviando datos',35,0,1)
display.show()

#Boton 1
btnA = DigitalInOut(board.D5)
btnA.direction = Direction.INPUT
btnA.pull = Pull.UP

#Boton 2
btnB = DigitalInOut(board.D6)
btnB.direction = Direction.INPUT
btnB.pull = Pull.UP

#Boton 3
btnC = DigitalInOut(board.D12)
btnC.direction = Direction.INPUT
btnC.pull = Pull.UP

# Configure Packet Radio
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm69 = adafruit_rfm69.RFM69(spi, CS, RESET, 433.0,sync_word=bytes([0xAA,0x2D,0xD4]))
# Potencia de la señal en dbm, el valor maximo para un RFM69HCW es de 20, con mayor consumo, para un RFM69 es 17
rfm69._tx_power = 17
rfm69.tx_power = 17
prev_packet = None
# Optionally set an encryption key (16 byte AES key). MUST match both
# on the transmitter and receiver (or be set to None to disable/the default).
# rfm69.encryption_key = b'\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08'



#Serial
bus = serial.Serial('/dev/serial0',baudrate=9600)


def bmm_update():
    x , y, z = dev_bmm150.read_mag_data()
    heading_rads = math.atan2(y,z)
    heading_deg = math.degrees(heading_rads)
    return x,y,z,heading_deg

def camera_update():
    if not btnA.value:
        display.fill(0)
        display.text("tomando foto",0,15,1)
        display.show()
        return camara.capture()
    return ""


#Fusion 9DOF

F = fusion.Fusion()
KYaw = fusion.Kalman()
KPitch = fusion.Kalman()
KRoll = fusion.Kalman()
mm = bmm_update()
#Por como esta acomodado el magnetometro en el cansat, esta es la orientacion adecuada
MM = np.array([[mm[1]],[mm[2]],[mm[0]]])

#Actualizar datos
mpu.update()

ACC = np.array([[mpu.accel[0]],[mpu.accel[1]],[mpu.accel[2]]])
GG = np.array ([[mpu.gyro[0]],[mpu.gyro[1]],[mpu.gyro[2]]])

#Convertirlos a yaw, pitch, roll
F.Accelerometer(ACC)
F.Magnetometer(MM,F.pitch,F.roll)
# F.Gyroscope(MPU.angularVelocity)
#Inicializar el filtro Kalman con el pitch y roll del acelerometro
KPitch.setAngle(F.pitch)
KRoll.setAngle(F.roll)
start = time.time();

#Formato:
#{"T1":"temp bmp","P":"press","A":"altitude","T":"tiempo","h": "humedad relativa", "T2":"Temperatura del sht", "Ax": "AccX","Ay": "AccY","Az": "AccZ","Wx":"GyroX" ....}

diccionario = {
    "T1":"0", # temp bmp °C
    "T2":"0", # temp sht °C
    "T3":"0", # temp mpu °C
    "P":"0", # Press hPa
    "A":"0", # Altitud m
    "T":"0", # Tiempo (en segundos con respecto al EPOCH) s
    "H":"0", # Humedad relativa %
    "Ax":"0", #AccelX m/s2
    "Ay":"0", #AccelY m/s2
    "Az":"0", #AccelZ m/s2
    "Wx":"0", #GyroX °/s
    "Wy":"0", #GyroY °/s
    "Wz":"0", #GyroZ °/s
    "Mx":"0", #MagnetX uT
    "My":"0", #MagnetY uT
    "Mz":"0", #MagnetZ uT
    "head":"0", #Magnet Heading °
    "Yaw":"0",
    "Pitch":"0",
    "Roll":"0",
    "Dt":"0"


}
f = open("data.log","a")
bus.write(bytes(b"Serial Correct\n"))

n=0
total_kb=0
while True:
    n+=1
    mpu.update()
    camera_update()
    magnet = bmm_update()
    foto = camera_update()
    if (foto != ""):
        try:
            #foto = camera_update()
            #raw = open(foto,"rb").readall()
            print (f"ahi te van {len(foto)} B de datos")
            print (foto)
        except:
            print("PHOTO SEND ERROR")
            display.fill(0)
            display.text("Photo ERROR",0,15,1)
            display.show()
            time.sleep(1)
            
            

    end = time.time()
    delta = end-start
    start = end
    mm = bmm_update()
    MM = np.array([[mm[1]],[mm[2]],[mm[0]]])
    ACC = np.array([[mpu.accel[0]],[mpu.accel[1]],[mpu.accel[2]]])
    GG = np.array ([[mpu.gyro[0]],[mpu.gyro[1]],[mpu.gyro[2]]])
    F.Accelerometer(ACC)

    # F.Gyroscope(MPU.angularVelocity)
    KPitch.getAngle(F.pitch,GG[1,0],delta)
    KRoll.getAngle(F.roll,GG[0,0],delta)


    F.Magnetometer(MM,KPitch.angle,KRoll.angle)


    KYaw.getAngle(F.yaw,GG[2,0],delta)
    # player.rotation_x = F.pitch
    # player.rotation_y = F.roll
    print (f"A_pitch: {F.pitch:.2f}, A_Roll: {F.roll:.2f}....Pitch: {KPitch.angle:.2f}, Roll: {KRoll.angle:.2f}, dt: {delta:.4f}")



    diccionario["T1"] = f"{bmp280.temperature:.2f}"
    diccionario["T2"] = f"{sht30.temperature[0]:.2f}"
    diccionario["T3"] = f"{mpu.tempC:.2f}"

    diccionario["T"] = f"{(time.time()-INIT_TIME):.3f}"

    diccionario["A"] = f"{bmp280.altitude:.2f}"
    diccionario["P"] = f"{bmp280.pressure:.2f}"
    diccionario["H"] = f"{sht30.relative_humidity[0]:.2f}"

    diccionario["Ax"] = f"{mpu.accel[0]:.2f}"
    diccionario["Ay"] = f"{mpu.accel[1]:.2f}"
    diccionario["Az"] = f"{mpu.accel[2]:.2f}"

    diccionario["Wx"] = f"{mpu.gyro[0]:.3f}"
    diccionario["Wy"] = f"{mpu.gyro[1]:.3f}"
    diccionario["Wz"] = f"{mpu.gyro[2]:.3f}"

    diccionario["Mx"] = f"{magnet[0]:.3f}"
    diccionario["My"] = f"{magnet[1]:.3f}"
    diccionario["Mz"] = f"{magnet[2]:.3f}"

    diccionario["head"] = f"{magnet[3]:.3f}"

    diccionario["Yaw"] = f"{KYaw.angle:.2f}"
    diccionario["Pitch"] = f"{KPitch.angle:.2f}"
    diccionario["Roll"] = f"{KRoll.angle:.2f}"
    diccionario["Dt"] = f"{(1000*(delta)):.1f}"

    
    utf = f"{diccionario}\n\r"
    bus.write(utf.encode())
    size_kb = len(utf)/1024
    total_kb += size_kb

    rf_data = bytes(utf,"utf-8")
    c = 0
    start = time.time()
    while (c < len(utf)):
        rfm69.send(rf_data[c:c+59])
        end = time.time()
        #print (f"Enviados 60 bytes en {(end-start):.2f} sec, ({(60/(end-start)):.2f}) kbps")
        start = end
        c+=58
        time.sleep(DELAY)
    # rfm69.send(bytes([10,13]))
    end = time.time()
    print(f"Enviados {size_kb} kilobytes de datos por RF y Serial en {(end-start):.3f} sec, {((size_kb)/(end-start)):.3f} kbps")

    print(utf)
    display.fill(0)
    display.text(f"{n} paquetes enviados",0,0,1)
    display.text(f" ({total_kb:.2f}) kb",0,10,1)

    display.show()
    f.write(utf)


    # print("\nTemperature: %0.1f C" % bmp280.temperature)
    # print("Pressure: %0.1f hPa" % bmp280.pressure)
    # print("Altitude = %0.2f meters" % bmp280.altitude)





# # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
# sensor = adafruit_sht31d.SHT31D(i2c)

# print("\033[1mSensor\033[0m = SHT31-D")
# print("\033[1mSerial Number\033[0m = ", sensor.serial_number, "\n")
# sensor.frequency = adafruit_sht31d.FREQUENCY_1
# sensor.mode = adafruit_sht31d.MODE_PERIODIC
# while True:
#     for i in range(3):
#         print("Please wait...", end="\r")
#         if i == 2:
#             sensor.heater = True
#         if i == 1:
#             time.sleep(4)
#             print("\033[91mCache half full.\033[0m")
#         else:
#             time.sleep(8)
#         if sensor.heater:
#             print("\033[1mHeater:\033[0m On    ")
#             sensor.heater = False
#         print("\033[1mTemperature:\033[0m ", sensor.temperature)
#         if not sensor.heater:
#             print("\033[1mHeater:\033[0m Off")
#         print("\033[1mHumidity:\033[0m ", sensor.relative_humidity, "\n")

#     time.sleep(2)
# sensor.mode = adafruit_sht31d.MODE_SINGLE

