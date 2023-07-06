import serial
import math
import numpy as np
import time
import ursina as ur

GRAVITY = np.array([[0],[0],[9.81]])
class Motion:
    def rotationMatrix(W:np.ndarray, rotMatrix:np.ndarray, delta):
        #R = np.zeros(shape=(3,3),dtype=np.float64)
        wx = W[0][0]
        wy = W[1][0]
        wz = W[2][0]
        S = np.array([
            [0,-wz,wy],
            [wz,0,-wx],
            [-wy,wx,0]
            ])
        _R = np.matmul(rotMatrix,S) * delta
        R = np.add(rotMatrix,  _R)
        #Analyze and seek for impossible events
        # R[0][0] = np.
        # R[1][0]
        # R[2][0]
        #R = np.clip(R,-1,1)
        return R
    def acceleration(a:np.ndarray,rotMatrix:np.ndarray,G:np.ndarray):
        return np.add(np.matmul(rotMatrix,a), G)
    def velocity(a:np.ndarray,v0:np.ndarray,delta):
        return np.add(a*delta, v0)
    def position(v:np.ndarray,p0:np.ndarray,delta):
        return np.add(v*delta ,p0)
    def angularPosition(W:np.ndarray,ap0:np.ndarray,delta):
        return np.add(W*delta,ap0)
    def rotationMatrix(pa: np.ndarray):
        a = pa[0][0]
        b = pa[1][0]
        y = pa[2][0]
        return np.array([
            [
            math.cos(a)*math.cos(b),
            math.cos(a)*math.sin(b)*math.sin(y)-math.sin(a)*math.cos(y),
            math.cos(a)*math.sin(b)*math.cos(y)+math.sin(a)*math.sin(y)
            ],
            [
            math.sin(a)*math.cos(b),
            math.sin(a)*math.sin(b)*math.sin(y)+math.cos(a)*math.cos(y),
            math.sin(a)*math.sin(b)*math.cos(y)-math.cos(a)*math.sin(y)
            ],
            [
            -math.sin(b),
            math.cos(b)*math.sin(y),
            math.cos(b)*math.cos(y)
            ]
            ])



# R = Motion.rotationMatrix(np.array([[0],[0],[0]]),np.array([[1,0,0],[0,1,0],[0,0,1]]),0.1)
# A = Motion.acceleration(np.array([[2.0],[3.0],[0.0]]),R,GRAVITY)

mpu = serial.Serial(baudrate=38400,port="COM4",bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)


# Ultima calibracion
# 12:52:19.455 -> averaging 10000 readings each time
# 12:52:21.954 -> .................... [-1657,-1656] --> [-2,14]	[-1077,-1075] --> [-13,2]	[1515,1517] --> [16364,16385]	[80,81] --> [0,5]	[34,35] --> [-1,2]	[45,46] --> [0,5]
# 12:53:12.000 -> ....................	XAccel			YAccel				ZAccel			XGyro			YGyro			ZGyro
# 12:53:59.526 ->  [-1657,-1656] --> [-3,14]	[-1077,-1076] --> [-13,2]	[1516,1517] --> [16384,16385]	[80,81] --> [0,5]	[34,35] --> [-1,2]	[45,46] --> [0,5]
# 12:54:02.118 -> .................... [-1657,-1656] --> [-3,14]	[-1077,-1076] --> [-15,2]	[1516,1517] --> [16381,16385]	[80,81] --> [0,5]	[34,35] --> [-1,2]	[45,45] --> [0,1]
# 12:54:49.703 -> -------------- done --------------
# offsets = np.array([
#             [-1657,80],
#            [-1077,34],
#            [1516,45]])

#mpu.open()
class MPU:  
    def __init__(self, COM: serial.Serial,offset = None) -> None:
        self.mpu = COM
        self.all_data = ""
        self.started = False
        self.ended = False
        self.accFactor = -GRAVITY[2][0]
        self.wFactor = math.pi/180
        self.accFactorRaw = 0
        self.wFactorRaw = 0
        self._accelerationRaw = np.array([[0],[0],[0]],dtype=np.int16)
        self._accelerationG = np.array([[0],[0],[0]],dtype=np.float16)
        self.acceleration = np.array([[0],[0],[0]],dtype=np.float16)

        self._angularVelocityRaw = np.array([[0],[0],[0]],dtype=np.int16)
        self._angularVelocityDeg = np.array([[0],[0],[0]],dtype=np.float16)
        self.angularVelocity = np.array([[0],[0],[0]],dtype=np.float16)
        self.minGyro = 0
        #Offsets:
        #    [AccX, GyroX]
        #    [AccY, GyroY]
        #    [AccZ, GyroZ]
        self.offsets = None
        if(offset is None):
            self.offsets = np.array([[0,0],[0,0],[0,0]])
        else:
            self.offsets = offset
    def update(self):
        self.ended = False
        while not self.ended:
            while (mpu.in_waiting==0):
                # buff = mpu.read_all()
                # mpu.flushInput()
                # mpu.flushOutput()
                continue
            for j in range (0,mpu.in_waiting):
                raw = chr(mpu.read(1)[0])

                if not self.started and raw != "^":
                    continue
                elif not self.started and raw == "^":
                    self.started = True
                    self.ended = False
                    continue
                elif self.started and raw == "$":
                    self.started = False
                    self.ended = True
                    break
                if self.started:
                    string = raw
                    self.all_data += string
        #Analizar datos
        
        #print(self.all_data)
        # print ("##")
        s = self.all_data.split("\n")
        for item in s:
            # print (item)
            if not item.startswith(("A:","W:","X:","Y:","Z:","Wx:","Wy:","Wz:")):
                continue
            try:
                name, val = item.split(" ")
            except:
                continue
            if name:
                if name == "A:":
                    self.accFactorRaw = int(val) 
                elif name == "W:":
                    self.wFactorRaw = float(val)
                elif name == "X:":
                    self._accelerationG[0][0] = float(val) + self.offsets[0][0]
                elif name == "Y:":
                    self._accelerationG[1][0] = float(val) + self.offsets[1][0]
                elif name == "Z:":
                    self._accelerationG[2][0] = float(val) + self.offsets[2][0]
                elif name == "Wx:":
                    self._angularVelocityDeg[0][0] = float(val) + self.offsets[0][1]
                    if (abs(float(val)) < self.minGyro):
                        self._angularVelocityDeg[0][0] = 0.0
                elif name == "Wy:":
                    self._angularVelocityDeg[1][0] = float(val) + self.offsets[1][1]
                    if (abs(float(val)) < self.minGyro):
                        self._angularVelocityDeg[1][0] = 0.0
                elif name == "Wz:":
                    self._angularVelocityDeg[2][0] = float(val) + self.offsets[2][1]
                    if (abs(float(val)) < self.minGyro):
                        self._angularVelocityDeg[2][0] = 0.0
        #convertir a m/s2 y rad/s
        self.all_data = ""
        self.acceleration = self._accelerationG * self.accFactor 
        self.angularVelocity = self._angularVelocityDeg * self.wFactor 
        # self._accelerationG *= 0
        # self._angularVelocityDeg *= 0
        
D = MPU(mpu)

R0 = np.array([[1,0,0],[0,1,0],[0,0,1]])
V0 = np.array([[0],[0],[0]])
P0 = np.array([[0],[0],[0]])
AP0 = np.array([[0],[0],[0]])
#print(D.accFactor)



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

samples = np.zeros((6,100))
# data = np.zeros((10000,4))
#mpu.write(0xb1)

for i in range (0,100):
    D.update()
    print (f"Sample {i}", end='\r')
    samples[0][i] = D.acceleration[0][0]
    samples[1][i] = D.acceleration[1][0]
    samples[2][i] = D.acceleration[2][0]

    samples[3][i] = D.angularVelocity[0][0]
    samples[4][i] = D.angularVelocity[1][0]
    samples[5][i] = D.angularVelocity[2][0]
offsets = np.array([
    np.average(samples[0]),np.average(samples[1]),
    np.average(samples[2]),np.average(samples[3]),
    np.average(samples[4]),np.average(samples[5])])

del samples
# for i in range (0,10000):
#     D.update()
#     print(f"Acc [m/s2]:{D.acceleration}")
#     print(f"W[rad/s]:{D.angularVelocity}")

radDeg = 180/math.pi
start = time.time()
# I=0
def update():
    global D,start,end,P0,R0,V0,AP0,offsets
    try:
        D.update()
    except:
        mpu.close()
        print("Error")
        exit()


    D.acceleration[0][0] -= offsets[0]
    D.acceleration[1][0] -= offsets[1]
    D.acceleration[2][0] = D.acceleration[2][0] - offsets[2] - GRAVITY[2][0]
    
    D.angularVelocity[0][0] -= offsets[3]
    D.angularVelocity[1][0] -= offsets[4]
    D.angularVelocity[2][0] -= offsets[5]
    

    end = time.time()
    delta = end-start
    start = end

    AP = Motion.angularPosition(D.angularVelocity,AP0,delta)
    AP0 = AP

    R = Motion.rotationMatrix(AP)
    error = np.linalg.det(R)
    
    print (error)    
    R0 = R
    A = Motion.acceleration(D.acceleration,R,GRAVITY)
    acc.position = A * 3 
    if (np.linalg.norm(A) < 0.07):
        A *= 0
    V = Motion.velocity(A,V0,delta)
    vel.position = V*3
    V0 = V
    P = Motion.position(V,P0,delta)
    P0 = P
    player.rotation = (AP[0][0]*radDeg,AP[1][0]*radDeg,AP[2][0]*radDeg)
    #player.set_position((P[0][0],P[1][0],P[2][0]))

    particlePos = np.transpose(np.array(particle.position))
    rotation = np.matmul(R,np.transpose(particlePos))
    particle.set_position((rotation[0],rotation[1],rotation[2]))

    print(f"Acc [m/s2]:{D.acceleration}")
    print(f"W[rad/s]:{D.angularVelocity}")
    print(f"Rotation matrix: {R}")
    print(f"Acceleration Vector: {A}")
    print(f"Estimated velocity: {V}")
    print(f"Estimated position: {P}")
#     #time.sleep(all_data[I][3])
#     #ur.time.sleep(all_data[I][3])
#     print (all_data[I][0])
#     I+=1
#     if (I>10000):
#         exit()
    
#     #time.sleep()
#     #player.x -= 1 * time.dt

app.run()
mpu.close()









    
mpu.close()