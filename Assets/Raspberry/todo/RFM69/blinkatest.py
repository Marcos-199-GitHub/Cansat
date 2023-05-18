import board 
import digitalio
import busio

print ("Hello blinka!")

#Try to get Digital In
pin = digitalio.DigitalInOut(board.D4)
print ("Digital IO OK")

#I2C
i2c = busio.I2C(board.SCL,board.SDA)
print ("I2C OK")

#SPI
spi = busio.SPI(board.SCLK,board.MOSI,board.MISO)
print("SPI ok\nDone")
