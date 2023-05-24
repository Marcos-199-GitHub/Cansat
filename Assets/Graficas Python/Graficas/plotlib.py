import serial
import numpy as np
import matplotlib.pyplot as plt
import time
import json


serial_buffer = ""
serial_line_complete = False
cerialconleche = serial.Serial('COM4', 9600)  

#diccionario = {'p': 0, 'h': 0, 't' : 0, 'hdehumildad': 0, '.': 0}
Presion = np.array([])
Temperatura = np.array([])
Humedad = np.array([])
Altura = np.array([])
Tiempo = np.array([])
i = 0


"""
diccionario = {
    "T1":"0", # temp bmp °C
    "T2":"0", # temp sht °C
    "P":"0", # Press hPa
    "A":"0", # Altitud m
    "T":"0", # Tiempo (en segundos con respecto al EPOCH) s
    "H":"0" # Humedad relativa %
}
"""
diccionario = {
    "T1":"0", # temp bmp °C
    "T2":"0", # temp sht °C
    "P":"0", # Press hPa
    "A":"0", # Altitud m
    "T":"0", # Tiempo (en segundos con respecto al EPOCH) s
    "H":"0" # Humedad relativa %
}



plt.ion()
while True : 
    #cerialconagua = cerialconleche.readline()  #Sugiero mandar de la siguiente manera los datos ['p', ..., '.'] 
    cut = []
    for i in range(0,10):
        cut.append( cerialconleche.readline())
    for line in cut:
        #cerialconagua = cerialconleche.readline()  #Sugiero mandar de la siguiente manera los datos ['p', ..., '.'] 
        try:
            line = str(line)[2:-3]
            line = line.replace("'",'"')
            diccionario = json.loads(line)
        except:
            #print (cerialconagua)
            print("Dato perdido, incompleto")
            continue

        try:
            Presion = np.append(Presion,float(diccionario["P"]))
            Temperatura = np.append(Temperatura,float(diccionario["T1"]))
            Humedad = np.append(Humedad,float(diccionario["H"]))
            Altura = np.append(Altura,float(diccionario["A"]))
            Tiempo = np.append(Tiempo,float(diccionario["T"]))
        except:
            print("Dato perdido, corrupto")
            continue
    plt.subplot(2,1,1)
    plt.plot(Tiempo,Temperatura)
    # plt.subplot(2,2,2)
    # plt.plot(Tiempo,Presion)
    plt.subplot(2,1,2)
    plt.plot(Tiempo,Humedad)
    # plt.subplot(2,2,4)
    # plt.plot(Tiempo,Altura)


    plt.draw()
    plt.pause(0.0001)
    plt.clf()


