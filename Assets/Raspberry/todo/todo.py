#Sistema
from threading import Thread
import time
import datetime
import os
import subprocess
import math
import sys
import numpy as np
import serial
#IO
import board
import busio
from digitalio import DigitalInOut, Direction, Pull
#Sensores
import adafruit_sht31d
import adafruit_bmp280
import mpu6050
import bmm150
# Radiofrecuencia
import adafruit_rfm69
#Pantalla del RFM
import adafruit_ssd1306
#GPS
from pynmeagps import NMEAReader
from ast import parse
#Camara
import camara
import compressor
#Fusion de los sensores de movimiento
sys.path.append('../../Graficas Python/Calibracion de los IMU')
import fusion
#Colores en terminal
from colorama import Fore, Back, Style


DELAY = 0.00
SHT30_ADDRESS = 0x44
INIT_TIME = time.time()
FORMATO = 1 #0 para json, 1 para tamaño optimizado
# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()  # uses board.SCL and board.SDA

###LCD
##reset_pin = DigitalInOut(board.D4)
##display = adafruit_ssd1306.SSD1306_I2C(128,32,i2c,reset = reset_pin)
###Clear display
##display.fill(0)
##display.show()
##width = display.width
##height = display.height

##display.text('CanSat UPIIH',35,0,1)
##display.show()


time.sleep(0.5)

##display.fill(0)
bmp280 = None
sht30 = None
dev_bmm150 = None
mpu = None
try:
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)
except:
    print ("BMP280 can't be initialized ")
try:
    sht30 = adafruit_sht31d.SHT31D(i2c)
except:
    print ("SHT30 can't be initialized ")
try:
    dev_bmm150 = bmm150.BMM150()
except:
    print ("BMM150 can't be initialized ")
try:
    mpu = mpu6050.MPU(i2c)
except:
    print ("MPU can't be initialized ")



#Configuraciones extra
# change this to match the location's pressure (hPa) at sea level
try:
    bmp280.sea_level_pressure = 1013.25
except:
    print("Can't configure BMP")
try:
    print("\033[1mSerial Number\033[0m = ", sht30.serial_number, "\n")
    sht30.frequency = adafruit_sht31d.FREQUENCY_1
    sht30.mode = adafruit_sht31d.MODE_PERIODIC
except:
    print("Can't configure SHT")
try:
    print (f"MPU correct: {mpu.checkId()}, id: {mpu.id}")
except:
    print("MPU failing")



#Botones
btnA = DigitalInOut(board.D5)
btnA.direction = Direction.INPUT
btnA.pull = Pull.UP
btnB = DigitalInOut(board.D6)
btnB.direction = Direction.INPUT
btnB.pull = Pull.UP
btnC = DigitalInOut(board.D12)
btnC.direction = Direction.INPUT
btnC.pull = Pull.UP
# Radio
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm69 = None
prev_packet = None
try:
    rfm69 = adafruit_rfm69.RFM69(spi, CS, RESET, 433.0,sync_word=bytes([0xAA,0x2D,0xD4]))
    # Potencia de la señal en dbm, el valor maximo para un RFM69HCW es de 20, con mayor consumo, para un RFM69 es 17
    rfm69._tx_power = 17
    rfm69.tx_power = 17
    prev_packet = None
    print("RFM listo")
except:
    print("RFM failing")
#GPS
gps = serial.Serial('/dev/serial0',baudrate=9600,timeout = 1.0) #1 segundo de timeout, si no, ante cualquier error se congela todo el programa
nmr = NMEAReader(gps)
#Camara
try:
    camara.initCamera()
except:
    print("Error iniciando camara")


def bmm_update():
    x , y, z = dev_bmm150.read_mag_data()
    heading_rads = math.atan2(y,z)
    heading_deg = math.degrees(heading_rads)
    return x,y,z,heading_deg

def camera_update():
    if not btnA.value:
        ##display.fill(0)
        ##display.text("tomando foto",0,15,1)
        ##display.show()
        capture = camara.capture()
        try:
            compressed = compressor.compress_img_bytes(capture,1,30,Gray = False,lowBits = 1)
            return compressed
        except:
            print ("Error comprimiendo")
            return capture
    return bytes(0)

MAX_GPS_TIMEOUT_EXCEED = 5
_GPS_TIMEOUT = 0
def gps_update():
    global _GPS_TIMEOUT, MAX_GPS_TIMEOUT_EXCEED
    gps_dict = {}
    #Si el tiempo de espera maximo ya se excedio 5 veces, dejar de leer el GPS para no perder tiempo
    if (_GPS_TIMEOUT >= MAX_GPS_TIMEOUT_EXCEED):
        return gps_dict
    a = time.time()
    (gps_raw_data, gps_parsed_data) = nmr.read()
    diff = time.time() - a
    #1 es el tiempo de timeout, esta condicional mide si el gps no se pudo leer
    if (diff >= 1):
        _GPS_TIMEOUT+=1
        print(f"Max GPS tiemout exceeded {_GPS_TIMEOUT} times")
    if (str(gps_raw_data).find("$GPRMC") > 0):
        gps_dict = gps_parsed_data.__dict__
        print(f"{Fore.CYAN}{gps_dict}{Style.RESET_ALL}")
    elif (str(gps_raw_data).find("$GPVTG") > 0):
        gps_dict = gps_parsed_data.__dict__
        print(f"{Fore.CYAN}{gps_dict}{Style.RESET_ALL}")
    elif (str(gps_raw_data).find("$GPGSA") > 0):
        gps_dict = gps_parsed_data.__dict__
        print(f"{Fore.CYAN}{gps_dict}{Style.RESET_ALL}")
    return gps_dict

#Fusion 9DOF
F = fusion.Fusion()
KYaw = fusion.Kalman()
KPitch = fusion.Kalman()
KRoll = fusion.Kalman()
#Actualizar datos
mm = None
try:
    mm = bmm_update()
    mpu.update()
    #Por como esta acomodado el magnetometro en el cansat, esta es la orientacion adecuada
    MM = np.array([[mm[1]],[mm[2]],[mm[0]]])
    ACC = np.array([[mpu.accel[0]],[mpu.accel[1]],[mpu.accel[2]]])
    GG = np.array ([[mpu.gyro[0]],[mpu.gyro[1]],[mpu.gyro[2]]])

    #Convertirlos a yaw, pitch, roll
    F.Accelerometer(ACC)
    F.Magnetometer(MM,F.pitch,F.roll)
    #Inicializar el filtro Kalman con el pitch y roll del acelerometro
    KPitch.setAngle(F.pitch)
    KRoll.setAngle(F.roll)
except:
    print("Can't update Angles")

def angles_update(accel, gyro, magnet, delta):
    MM = np.array([[magnet[1]],[magnet[2]],[magnet[0]]])
    ACC = np.array([[accel[0]],[accel[1]],[accel[2]]])
    GG = np.array ([[gyro[0]],[gyro[1]],[gyro[2]]])
    F.Accelerometer(ACC)

    KPitch.getAngle(F.pitch,GG[1,0],delta)
    KRoll.getAngle(F.roll,GG[0,0],delta)
    F.Magnetometer(MM,KPitch.angle,KRoll.angle)
    KYaw.getAngle(F.yaw,GG[2,0],delta)
    print (f"A_pitch: {F.pitch:.2f}, A_Roll: {F.roll:.2f}....Pitch: {KPitch.angle:.2f}, Roll: {KRoll.angle:.2f}, dt: {delta:.4f}")
    return KYaw.angle,KPitch.angle,KRoll.angle



#Archivo de log
f = open("data.log","a")

start = time.time();
n=0
total_kb=0
globalStart = time.time()
speed = 0
#Diccionario de datos
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


def lcd_update():
    global n, total_kb, globalStart, speed
    last_n = n
    while True:
        if (last_n!=n):
            last_n = n
            ##display.fill(0)
            ##display.text(f"{n} paquetes ({total_kb:.2f}Kb)",0,0,1)
            ##display.text(f"",0,10,1)
            ##display.text(f"{speed:.0f}Kb/s,{(total_kb/(time.time()-globalStart)):.2f} Kb/s avg",0,10,1)
            ##display.show()


def main():
    global n, total_kb, globalStart, speed, diccionario, start, f, end
    print("Main")
    f.write(f"{datetime.datetime.fromtimestamp(datetime.datetime.now().timestamp() - 7*3600).strftime('%c')}\n")
    f.close()
    f = open("data.log","a")
    extT = 0
    relH = 0
    magnet = (0,0,0,0)
    _accel = [0,0,0]
    _gyro = [0,0,0]
    altitude = 0
    pressure = 0
    while True:
        n+=1 
        utf = ""

        s=time.time()
        try:
             mpu.update()
             _accel = mpu.accel() 
             _gyro = mpu.gyro()
        except:
            print ("Can't update MPU")
        try:
            magnet = bmm_update()
        except:
            print ("Can't update BMM")
        try:
            extT = sht30.temperature[0]
            relH = sht30.relative_humidity[0]
        except:
            print("Can't update SHT")

        foto = camera_update()
        e = time.time()
        gps_dict = gps_update()
        #Nota: tomar medidas del bmp toma mucho tiempo
        try:
            pressure = bmp280.pressure
            altitude = 44330 * (1.0 - math.pow(pressure / bmp280.sea_level_pressure, 0.1903))
        except:
            print ("Can't update BMP")

        print (f"\n{Fore.RED}{(e-s):.2f} s en actualizar sensores y gps{Style.RESET_ALL}")
        s = e

        end = time.time()
        delta = end-start
        start = end
        yaw,pitch,roll = 0,0,0
        try:
            yaw, pitch, roll = angles_update(mpu.accel,mpu.gyro,magnet,delta)
        except:
            print("Can't update angles")
        
        e = time.time()
        print (f"{Fore.RED}{(e-s):.2f} s en actualizar angulos{Style.RESET_ALL}")
        s = e
        ##diccionario["T1"] = f"{bmp280.temperature:.2f}"
        tempInterna = float(subprocess.check_output(["vcgencmd", "measure_temp"])[5:-3])

        diccionario["T2"] = f"{extT:.2f}"
        diccionario["T3"] = f"{tempInterna:.2f}"

        diccionario["T"] = f"{(time.time()-INIT_TIME):.3f}"

        diccionario["A"] = f"{altitude:.2f}"
        diccionario["P"] = f"{pressure:.2f}"
        diccionario["H"] = f"{relH:.2f}"

        diccionario["Ax"] = f"{_accel[0]:.2f}"
        diccionario["Ay"] = f"{_accel[1]:.2f}"
        diccionario["Az"] = f"{_accel[2]:.2f}"

        diccionario["Wx"] = f"{_gyro[0]:.3f}"
        diccionario["Wy"] = f"{_gyro[1]:.3f}"
        diccionario["Wz"] = f"{_gyro[2]:.3f}"

        diccionario["Mx"] = f"{magnet[0]:.3f}"
        diccionario["My"] = f"{magnet[1]:.3f}"
        diccionario["Mz"] = f"{magnet[2]:.3f}"

        diccionario["He"] = f"{magnet[3]:.3f}"

        diccionario["Y"] = f"{yaw:.2f}"
        diccionario["P"] = f"{pitch:.2f}"
        diccionario["R"] = f"{roll:.2f}"

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
            
        diccionario['Im'] = len(foto)

        e = time.time()
        print(f"{Fore.RED}{(e-s):.3f} s en actualizar el diccionario{Style.RESET_ALL}")
        s = e

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
            print (ut)
        
        size_kb = len(utf)/1024 + len(foto)/1024
        total_kb += size_kb

        rf_data = bytes(utf,"utf-8")
        c = 0

        e = time.time()
        print(f"{Fore.RED}{(e-s):.2f} s en convertir a bytes{Style.RESET_ALL}")
        try:
            s = e
            len_utf = len(utf)
            while (c < len_utf):
                rfm69.send(rf_data[c:c+59])
                c+=58
        except:
            print("Can't send data over RF")
        e = time.time()
        d = e-s
        print(f"{Fore.RED}{(e-s):.3f} s en enviar por RF{Style.RESET_ALL}")
        try:
            if (len(foto)>0):
                #aqui se va a enviar la imagen
                s = e
                c = 0
                while (c < (len(foto))):
                    rfm69.send(foto[c:c+59])
                    c+=58

        except:
            print("Can't send image over RF")

            e = time.time()
            print(f"{Fore.RED}{(e-s):.3f} s en enviar {Back.GREEN}imagen por RF{Style.RESET_ALL}")
        
        d += (e-s)
        speed = size_kb/d
        print(f"{Style.BRIGHT}Enviados {size_kb:.3f} Kb de datos por RF en {(d):.3f} s, {((size_kb)/(d)):.3f} Kb/s{Style.RESET_ALL}")
        print(f"{Fore.GREEN}{utf}{Style.RESET_ALL}",end = "")
        s = e
        ##display.fill(0)
        ##display.text(f"{n} paquetes ({total_kb:.2f}Kb)",0,0,1)
        ##display.text(f"",0,10,1)
        ##display.text(f"{speed:.0f}Kb/s,{(total_kb/(time.time()-globalStart)):.2f} Kb/s avg",0,10,1)
        ##display.show()
        e = time.time()
        print(f"{Fore.RED}{(e-s):.2f} s en actualizar el lcd{Style.RESET_ALL}")
        


        s = e
        f.write(utf)
        e = time.time()
        print(f"{Fore.RED}{(e-s):.2f} s en actualizar el archivo log{Style.RESET_ALL}")
        #time.sleep (1)


if __name__ == "__main__":
    #mainThread = Thread(target=main)
    #lcdThread =  Thread(target=lcd_update)
    #mainThread.start()
    #lcdThread.start()
    main()
    
