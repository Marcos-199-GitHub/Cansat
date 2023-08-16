
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
import compressor


#Fusion de los sensores de movimiento
import sys
sys.path.append('../../Graficas Python/Calibracion de los IMU')
import fusion
import numpy as np

#GPS
from pynmeagps import NMEAReader
from ast import parse

#Colores
from colorama import Fore, Back, Style

DELAY = 0.00
SHT30_ADDRESS = 0x44
INIT_TIME = time.time()
PHOTO_CMD = "streamer -d /dev/video0 -s 320x240 -o "
FORMATO = 1 #0 para json, 1 para tamaño optimizado
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
#rfm69.bitrate = 300000
prev_packet = None
# Optionally set an encryption key (16 byte AES key). MUST match both
# on the transmitter and receiver (or be set to None to disable/the default).
# rfm69.encryption_key = b'\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08'

#Camara
try:
    camara.initCamera()
except:
    print("Error iniciando camara")


#Serial
#GPS
gps = serial.Serial('/dev/serial0',baudrate=9600)


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
        capture = camara.capture()
        try:
            compressed = compressor.compress_img_bytes(capture,1,30,Gray = True)
            return compressed
        except:
            print ("Error comprimiendo")
            return capture
    return bytes(0)


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


f = open("data.log","a")

n=0
total_kb=0
nmr = NMEAReader(gps)

globalStart = time.time()
while True:

    #Formato:
    #{"T1":"temp bmp","P":"press","A":"altitude","T":"tiempo","h": "humedad relativa", "T2":"Temperatura del sht", "Ax": "AccX","Ay": "AccY","Az": "AccZ","Wx":"GyroX" ....}

    diccionario = {
    #"T1":"0", # temp bmp °C
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
    "He":"0", #Magnet Heading °
    "Y":"0", #Yaw
    "P":"0", #Pitch 
    "R":"0", #Roll
    "Dt":"0", #Delta T del filtro
    "Lt":"0", #Latitud
    "Lg":"0", #Longitud
    "Di":"NW", #Indica la direccion para la latitud y longitud respectivamente (No es necesario si se utilizan signos)
    "Km":"0", #Velocidad respecto al suelo
    "DP":"0", #Dilucion de precision del GPS
    "Im":"0", #Si el proximo mensaje va a ser una imagen, su valor es el tamaño en bytes, si no, es 0

    }


    n+=1
    s=time.time()
    mpu.update()
    #camera_update()
    magnet = bmm_update()
    foto = camera_update()
    mm = bmm_update()
    e = time.time()
    gps_dict = {}
#    print (gps.readline())
    (gps_raw_data, gps_parsed_data) = nmr.read()
    if (str(gps_raw_data).find("$GPRMC") > 0):
        gps_dict = gps_parsed_data.__dict__
        print(f"{Fore.CYAN}{gps_dict}{Style.RESET_ALL}")
        #print(gps_parsed_data.__dict__["alt"])
    if (str(gps_raw_data).find("$GPVTG") > 0):
        gps_dict = gps_parsed_data.__dict__
        print(f"{Fore.CYAN}{gps_dict}{Style.RESET_ALL}")
        #print(parsed_data.__dict__["alt"])
    if (str(gps_raw_data).find("$GPGSA") > 0):
        gps_dict = gps_parsed_data.__dict__
        print(f"{Fore.CYAN}{gps_dict}{Style.RESET_ALL}")
        #print(parsed_data.__dict__["alt"])
    print (f"\n{Fore.RED}{(e-s):.2f} s en actualizar sensores y gps{Style.RESET_ALL}")
    s = e
    end = time.time()
    delta = end-start
    start = end

    MM = np.array([[mm[1]],[mm[2]],[mm[0]]])
    ACC = np.array([[mpu.accel[0]],[mpu.accel[1]],[mpu.accel[2]]])
    GG = np.array ([[mpu.gyro[0]],[mpu.gyro[1]],[mpu.gyro[2]]])
    F.Accelerometer(ACC)

    # F.Gyroscope(MPU.angularVelocity)
    KPitch.getAngle(F.pitch,GG[1,0],delta)
    KRoll.getAngle(F.roll,GG[0,0],delta)


    F.Magnetometer(MM,KPitch.angle,KRoll.angle)


    KYaw.getAngle(F.yaw,GG[2,0],delta)
    print (f"A_pitch: {F.pitch:.2f}, A_Roll: {F.roll:.2f}....Pitch: {KPitch.angle:.2f}, Roll: {KRoll.angle:.2f}, dt: {delta:.4f}")

    
    e = time.time()
    print (f"{Fore.RED}{(e-s):.2f} s en actualizar angulos{Style.RESET_ALL}")
    s = e
    #Nota: tomar medidas del bmp toma mucho tiempo
    pressure = bmp280.pressure
    altitude = 44330 * (1.0 - math.pow(pressure / bmp280.sea_level_pressure, 0.1903))
    ##diccionario["T1"] = f"{bmp280.temperature:.2f}"
    diccionario["T2"] = f"{sht30.temperature[0]:.2f}"
    diccionario["T3"] = f"{mpu.tempC:.2f}"

    diccionario["T"] = f"{(time.time()-INIT_TIME):.3f}"

    diccionario["A"] = f"{altitude:.2f}"
    diccionario["P"] = f"{pressure:.2f}"
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

    diccionario["He"] = f"{magnet[3]:.3f}"

    diccionario["Y"] = f"{KYaw.angle:.2f}"
    diccionario["P"] = f"{KPitch.angle:.2f}"
    diccionario["R"] = f"{KRoll.angle:.2f}"

    diccionario["Dt"] = f"{(1000*(delta)):.1f}"

    try:
        diccionario["Lt"] = f"{(gps_dict['lat']):.6f}"
        diccionario["Lg"] = f"{(gps_dict['lon']):.6f}"
        diccionario["Di"] = f"{gps_dict['NS']}{gps_dict['EW']}"
    except:
        pass
    try:
        diccionario["Km"] = f"{(gps_dict['sogk']):.2f}"
    except:
        pass

    try:
        diccionario["DP"] = f"{(gps_dict['PDOP']):.2f}"
    except:
        pass
        
    if (len(foto) > 0):
        diccionario['Im'] = len(foto)
    e = time.time()

    print(f"{Fore.RED}{(e-s):.3f} s en actualizar el diccionario{Style.RESET_ALL}")


    s = e
    utf = ""
    if FORMATO == 0:
        utf = f"{diccionario}\n\r"
    elif FORMATO == 1:
        ut = f"{float(diccionario['T2'])},{float(diccionario['T3'])},{float(diccionario['T'])},"
        ut+= f"{float(diccionario['A'])},{float(diccionario['P'])},{float(diccionario['H'])},"
        ut+= f"{float(diccionario['Ax'])},{float(diccionario['Ay'])},{float(diccionario['Az'])},"
        ut+= f"{float(diccionario['Wx'])},{float(diccionario['Wy'])},{float(diccionario['Wz'])},"
        ut+= f"{float(diccionario['Mx'])},{float(diccionario['My'])},{float(diccionario['Mz'])},"
        ut+= f"{float(diccionario['He'])},{float(diccionario['Y'])},{float(diccionario['P'])},"
        ut+= f"{float(diccionario['R'])},{float(diccionario['Dt'])},{float(diccionario['Lt'])},"
        ut+= f"{float(diccionario['Lg'])},{float(diccionario['Km'])},{float(diccionario['DP'])},"
        ut+= f"{int(diccionario['Im'])}"

        utf = "{" + ut + "}\n\r"
#        ut+= f"{float(diccionario['P'])}}"
        print (ut)
    size_kb = len(utf)/1024
    total_kb += size_kb

    rf_data = bytes(utf,"utf-8")
    c = 0

    e = time.time()

    print(f"{Fore.RED}{(e-s):.2f} s en convertir a bytes{Style.RESET_ALL}")

    s = e
    len_utf = len(utf)
    while (c < len_utf):
        ss  = time.time()
        rfm69.send(rf_data[c:c+59])
        #e = time.time()
        #print (f"Enviados 60 bytes en {(1000*(e-ss)):.1f} ms, ({((60/1024)/(e-ss)):.2f}) Kb/s")
        #s = e
        c+=58
        #time.sleep(DELAY)
    # rfm69.send(bytes([10,13]))
    e = time.time()
    d = e-s
    print(f"{Fore.RED}{(e-s):.3f} s en enviar por RF{Style.RESET_ALL}")
    
    if (len(foto)>0):
        #aqui se va a enviar la imagen
        s = e
        c = 0
        while (c < (len(foto))):
            rfm69.send(foto[c:c+59])
            #e = time.time()
            #print (f"Enviados 60 bytes en {(1000*(e-ss)):.1f} ms, ({((60/1024)/(e-ss)):.2f}) Kb/s")
            #s = e
            c+=58
            #time.sleep(DELAY)
        e = time.time()
        print(f"{Fore.RED}{(e-s):.3f} s en enviar {Back.GREEN}imagen por RF{Style.RESET_ALL}")
        #print(f"{foto}")
        #input("Enter para continuar")

        

    print(f"{Style.BRIGHT}Enviados {size_kb:.3f} Kb de datos por RF en {(d):.3f} s, {((size_kb)/(d)):.3f} Kb/s{Style.RESET_ALL}")
    s = e
    print(f"{Fore.GREEN}{utf}{Style.RESET_ALL}",end = "")
    #display.fill(0)
    #display.text(f"{n} paquetes ({total_kb:.2f}Kb)",0,0,1)
    #display.text(f"",0,10,1)
    #display.text(f"{((size_kb)/(d)):.0f}Kb/s,{(total_kb/(time.time()-globalStart)):.2f} Kb/s avg",0,10,1)

    #display.show()
    e = time.time()
    print(f"{Fore.RED}{(e-s):.2f} s en el display{Style.RESET_ALL}")
    s = e
    f.write(utf)
    e = time.time()
    print(f"{Fore.RED}{(e-s):.2f} s en actualizar el archivo log{Style.RESET_ALL}")
    #time.sleep (1)

