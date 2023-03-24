# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Simpletest Example that shows how to get temperature,
   pressure, and altitude readings from a BMP280"""
import time
import board

# import digitalio # For use with SPI
import adafruit_bmp280

SHT30_ADDRESS = 0x44

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


def readSHT():
   #sht30
   # SHT30 address, 0x44(68)
   # Send measurement command, 0x2C(44)
   #		0x06(06)	High repeatability measurement
   i2c.writeto(SHT30_ADDRESS,bytes([0x2C]),stop=False)
   i2c.writeto(SHT30_ADDRESS,bytes([0x06]),stop=True)

   time.sleep(0.5)

   # SHT30 address, 0x44(68)
   # Read data back from 0x00(00), 6 bytes
   # cTemp MSB, cTemp LSB, cTemp CRC, Humididty MSB, Humidity LSB, Humidity CRC

   data = bytearray(6)
   i2c.writeto(SHT30_ADDRESS,bytes([0x00]),stop=False)
   i2c.readfrom_into(SHT30_ADDRESS,data)

   # Convert the data
   tempRaw = (data[0] << 8) | data[1]
   cTemp = (tempRaw* 175) - 45
   
   fTemp = cTemp * 1.8 + 32
   humidityRaw = (data[3] <<8) | data[4]
   humidity = 100 * humidityRaw / 65535

   # Output data to screen
   print ("Relative Humidity : %.2f %%RH" %humidity)
   print ("Temperature in Celsius : %.2f C" %cTemp)
   print ("Temperature in Fahrenheit : %.2f F" %fTemp)


while True:
    print("\nTemperature: %0.1f C" % bmp280.temperature)
    print("Pressure: %0.1f hPa" % bmp280.pressure)
    print("Altitude = %0.2f meters" % bmp280.altitude)
    readSHT()
    time.sleep(2)
