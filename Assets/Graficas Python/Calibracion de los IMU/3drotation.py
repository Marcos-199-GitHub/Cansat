
import ursina as ur
import calibracion
import fusion
import serial
import time

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

mpu = serial.Serial(baudrate=38400,port="COM4",bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
D = calibracion.MPU(mpu)
F = fusion.Fusion()
KYaw = fusion.Kalman()
KPitch = fusion.Kalman()
KRoll = fusion.Kalman()
#Actualizar datos
D.update()
#Convertirlos a yaw, pitch, roll
F.Accelerometer(D.acceleration)
# F.Gyroscope(MPU.angularVelocity)
#Inicializar el filtro Kalman con el pitch y roll del acelerometro
KPitch.setAngle(F.pitch)
KRoll.setAngle(F.roll)
start = time.time();


def update():
    global D,start,end,P0,R0,V0,AP0,offsets
    try:
        D.update()
    except:
        mpu.close()
        print("Error")
        exit()

    end = time.time()
    delta = end-start
    start = end
    F.Accelerometer(D.acceleration)
    # F.Gyroscope(MPU.angularVelocity)
    KPitch.getAngle(F.pitch,D.angularVelocity[1,0],delta)
    KRoll.getAngle(F.roll,D.angularVelocity[0,0],delta)
    player.rotation_x = KPitch.angle
    player.rotation_y = KRoll.angle
    # player.rotation_x = F.pitch
    # player.rotation_y = F.roll
    print (f"A_pitch: {F.pitch:.2f}, A_Roll: {F.roll:.2f}....Pitch: {KPitch.angle:.2f}, Roll: {KRoll.angle:.2f}, dt: {delta:.4f}")
    #player.set_position((P[0][0],P[1][0],P[2][0]))
D.clean()
app.run()
mpu.close()




app.run()


