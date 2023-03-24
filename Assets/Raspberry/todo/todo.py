# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_sht31d
import serial
# import digitalio # For use with SPI
import adafruit_bmp280
import grove_bmm150
import mpu6050

SHT30_ADDRESS = 0x44
INIT_TIME = time.time()

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

#print("\033[1mSensor\033[0m = SHT31-D")
#print("\033[1mSerial Number\033[0m = ", sht30.serial_number, "\n")
sht30.frequency = adafruit_sht31d.FREQUENCY_1
sht30.mode = adafruit_sht31d.MODE_PERIODIC

#MPU y BMM
bmm150 = grove_bmm150.BMM150(i2c)
print (f"BMM correct: {bmm150.checkId()}, id: {bmm150.id}")
mpu = mpu6050.MPU(i2c)
print (f"MPU correct: {mpu.checkId()}, id: {mpu.id}")



#Serial
bus = serial.Serial('/dev/serial0',baudrate=9600)

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

}
f = open("data.log","a")
bus.write(bytes(b"Serial Correct\n"))
while True:
    mpu.update()

    diccionario["T1"] = f"{bmp280.temperature}"
    diccionario["T2"] = f"{sht30.temperature[0]}"
    diccionario["T3"] = f"{mpu.tempC}"

    diccionario["T"] = f"{time.time()-INIT_TIME}"

    diccionario["A"] = f"{bmp280.altitude}"
    diccionario["P"] = f"{bmp280.pressure}"
    diccionario["H"] = f"{sht30.relative_humidity[0]}"

    diccionario["Ax"] = f"{mpu.accel[0]}"
    diccionario["Ay"] = f"{mpu.accel[1]}"
    diccionario["Az"] = f"{mpu.accel[2]}"

    diccionario["Wx"] = f"{mpu.gyro[0]}"
    diccionario["Wy"] = f"{mpu.gyro[1]}"
    diccionario["Wz"] = f"{mpu.gyro[2]}"
    
    utf = f"{diccionario}\n"
    bus.write(utf.encode())
    print(utf)
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

