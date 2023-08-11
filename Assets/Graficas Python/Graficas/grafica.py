import serial
import numpy as np
import matplotlib.pyplot as plt
import vpython as vp
from vpython import *
import time
import json


cerialconleche = serial.Serial('COM8', 9600)  

#diccionario = {'p': 0, 'h': 0, 't' : 0, 'hdehumildad': 0, '.': 0}
Presion = np.array([])
Temperatura = np.array([])
Humedad = np.array([])
Altura = np.array([])
Tiempo = np.array([])
TiempoCero = 0
i = 0

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
    "head": "0",
    "Yaw": "0",
    "Pitch":"0",
    "Roll": "0",

}
def crear_esfera(x,y,z):
    nueva_esfera = sphere(pos=vector(x, y, z), radius=0.5, color=color.blue)
#Interactive mode on
plt.ion()
scene = canvas(title='Esferas desde el puerto serie', width=800, height=600, center=vector(0,0,0), background=color.white)
n = crear_esfera(0,0,0)
time.sleep(1)
while True : 
    #cerialconagua = cerialconleche.readline()  #Sugiero mandar de la siguiente manera los datos ['p', ..., '.'] 
    cut = []
    #Leer n lineas del serial antes de guardarlas en el arreglo
    for i in range(0,5):
        cut.append(cerialconleche.readline())
    for line in cut:
        #cerialconagua = cerialconleche.readline()  #Sugiero mandar de la siguiente manera los datos ['p', ..., '.'] 
        try:
            line = str(line)[2:-3]
            line = line.replace("'",'"')
            print (line)
            diccionario = json.loads(line)
        except:
            #print (cerialconagua)
            print("Dato perdido, incompleto")
            continue
        try:

            T = float(diccionario["T"]) - TiempoCero
            if (TiempoCero==0):
                TiempoCero = T
                T = 0
            T1 = float(diccionario["T1"])
            P = float(diccionario["P"])
            H = float(diccionario["H"])
            A = float(diccionario["A"])

            Presion = np.append(Presion,P)
            Temperatura = np.append(Temperatura,T1)
            Humedad = np.append(Humedad,H)
            Altura = np.append(Altura,A)
            Tiempo = np.append(Tiempo,T)
            crear_esfera(T,T1,H)
        except:
            print("Dato perdido, corrupto")
            continue
    

    plt.subplot(2,1,1)
    plt.plot(Altura,Temperatura)
    plt.xlabel('Altura (s)')
    plt.ylabel('Temperatura BMP')
    # plt.subplot(2,2,2)
    # plt.plot(Altura,Presion)
    plt.subplot(2,1,2)
    plt.plot(Altura,Humedad)
    plt.xlabel('Altura (s)')
    plt.ylabel('Humedad')
    # plt.subplot(2,2,4)
    # plt.plot(Tiempo,Altura)
    #plt.legend()
    plt.draw()
    plt.pause(0.0001)
    plt.clf()



    # vp.graph(x=Presion[i], y=Altura[i], z=Temperatura[i])
    # vp.xlabel('Presión')
    # vp.ylabel('Altura')
    # vp.zlabel('Temperatura')
    # i += 1
    # vp.show()
     
    

    



