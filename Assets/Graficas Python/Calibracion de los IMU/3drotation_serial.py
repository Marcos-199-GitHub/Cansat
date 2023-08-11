
import ursina as ur
import serial
import time
import numpy as np
import json

# create a window
app = ur.Ursina()

# most things in ursina are Entities. An Entity is a thing you place in the world.
# you can think of them as GameObjects in Unity or Actors in Unreal.
# the first paramenter tells us the Entity's model will be a 3d-model called 'cube'.
# ursina includes some basic models like 'cube', 'sphere' and 'quad'.

# the next parameter tells us the model's color should be orange.

# 'scale_y=2' tells us how big the entity should be in the vertical axis, how tall it should be.
# in ursina, positive x is right, positive y is up, and positive z is forward.
origin = ur.Entity(model='sphere',color = ur.color.black,scale = (0.1,0.1,0.1))
xaxis = ur.Entity(model = 'cube', color = ur.color.red, scale = (20,0.05,0.05))
yaxis = ur.Entity(model = 'cube', color = ur.color.green, scale = (0.05,20,0.05))
zaxis = ur.Entity(model = 'cube', color = ur.color.blue, scale = (0.05,0.05,20))
player = ur.Entity(model='cube', color=ur.color.orange, scale_y=1)
particle = ur.Entity(model='sphere',color = ur.color.black,scale = (0.5,0.5,0.5))
acc = ur.Entity(model='sphere',color = ur.color.orange,scale = (0.5,0.5,0.5))
vel = ur.Entity(model='sphere',color = ur.color.blue,scale = (0.5,0.5,0.5))
particle.position = (3,0,0)
sun = ur.DirectionalLight()
sun.look_at(ur.Vec3(1,1,1))
ur.Sky()


ur.camera.position = (10,30,-55)
ur.camera.rotation_x = 30
ur.camera.rotation_z = 5
ur.camera.rotation_y = -10

ur.camera.fov = 300


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

def update():
    #cerialconagua = cerialconleche.readline()  #Sugiero mandar de la siguiente manera los datos ['p', ..., '.'] 
    cut = []
    Yaw = 0 
    Pitch = 0
    Roll = 0
    #Leer n lineas del serial antes de guardarlas en el arreglo
    for i in range(0,1):
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
            Yaw = float(diccionario["Yaw"]) 
            Pitch=  float(diccionario["Pitch"]) 
            Roll =  float(diccionario["Roll"]) 
        except:
            print("Dato perdido, corrupto")
            continue
    player.rotation_x = Pitch
    player.rotation_y = Roll
    player.rotation_z = Yaw
    print (f"Pitch: {Pitch:.2f}, Roll: {Roll:.2f}, Yaw: {Yaw:.2f}")


app.run()



