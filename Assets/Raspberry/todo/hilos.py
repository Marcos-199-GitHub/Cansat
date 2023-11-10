from threading import Thread

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
    "Pi":"0", #Pitch
    "R":"0", #Roll
    "Dt":"0", #Delta T del filtro
    "Lt":"0", #Latitud
    "Lg":"0", #Longitud
    "Di":"NW", #Indica la direccion para la latitud y longitud respectivamente (No es necesario si se utilizan signos)
    "Km":"0", #Velocidad respecto al suelo
    "DP":"0", #Dilucion de precision del GPS
    "Im":"0", #Si el proximo mensaje va a ser una imagen, su valor es el tamaño en bytes, si no, es 0
}
sensoresTerminados = False #cuando termine de leer
fotoTomada = False #cuando termine de tomar la foto
enviarFoto = False #cuando lo siguiente que debe enviar es una foto

def leerSensores():
    while True:
        pass

def tomarFoto():
    while True:
        pass

def guardarSensores():
    while True:
        pass

def enviarDatos():
    while True:
        if enviarFoto:
            enviarFoto = False
            #la envia
            pass

        if fotoTomada:
            fotoTomada = False
            diccionario["Im"] = 1
            enviarFoto = True
            #Envia el diccionario ya sea que tenga sensores actualizados o no
            pass

        elif sensoresTerminados:
            sensoresTerminados = False
            #envia el diccionario
            pass



def iniciarHilos():
    Thread(target=leerSensores).start()
    Thread(target=tomarFoto).start()
    Thread(target=guardarSensores).start()
    Thread(target=enviarDatos).start()


if __name__ == '__main__':
    iniciarHilos()