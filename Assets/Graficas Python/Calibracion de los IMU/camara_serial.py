import serial
import os
#from PIL import Image
from io import BytesIO
import cv2
import time 

datos = serial.Serial('COM8', 38400)  
datos.readline()
isImg = False
imgLen = 0
while True:
    if (not isImg):
        line = datos.readline()
        arr = (str(line)[3:-4]).split(',')
        try:
            imgLen = int(arr[24])
        except:
            imgLen = -1
        print (f"{arr}, {imgLen}")
        if (imgLen > 0):
            isImg = True
    else:
        print (f"{imgLen} bytes")
        rawImg = datos.read(imgLen)
        print (rawImg)
        print (len(rawImg))
        with open("test.jpg","wb") as f:
            f.write(rawImg)
        image = cv2.imread("test.jpg")
        cv2.imshow("CanSat",image)
        #Esperar un segundo
        time.sleep(1.5)
        cv2.destroyAllWindows()
        
        # bio = BytesIO(rawImg)
        # img = Image.open("test.jpg")
        # img.show()
        # img.save("Foto.jpeg")
        isImg = False
        imgLen = 0

