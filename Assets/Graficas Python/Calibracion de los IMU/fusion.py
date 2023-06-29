import numpy as np
#Fuentes:

# 1.- Data Fusion with 9 Degrees of Freedom Inertial Measurement Unit To Determine Object’s Orientation
# By
# Long Tran
# Senior Project
# Electrical Engineering Department
# California Polytechnic State University San Luis Obispo June 2017

# 2.- https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html

class Fusion:
    def __init__(self) -> None:
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.Groll = 0
        self.Gpitch = 0
        self.Gyaw = 0

        #Orden: [roll, pitch, yaw]
        #CFAngle: complementary Filter angle
        #TODO: inicializar el CFAngle al yaw, pitch y roll iniciales
        self.CFAngle = np.zeros((3,1))

        #Complementary filter High pass weigth
        #Normalmente se utiliza un valor de 0.98
        self.CF_HPWeigth = 0.98
        #Complementary filter Low pass weigth
        #DONT USE: it will be overriden by HPWeigth
        self.CF_LPWeigth = 0.02
        #HP + LP = 1
        self.KFilterYaw = Kalman()
        self.KFilterRoll = Kalman()
        self.KFilterPitch = Kalman()
        self.KFAngle = np.zeros((3,1))
        
    #Obtiene los valores de roll y pitch mediante los valores de aceleracion (ya deberian estar calibrados)
    def Accelerometer(self,A:np.ndarray):
        # roll = atan(ay/az)
        self.roll = math.atan2(A[1,0], A[2,0])
        # pitch = atan(-ax/sqrt(ay**2+az**2))
        # math.hypot is hypotenuse
        self.pitch = math.atan2(-A[0,0],math.hypot(a[1,0],a[2,0]))
    def Magnetometer(self,M:np.ndarray,pitch,roll):
        Mx = M[0,0]*math.cos(pitch) + m[2,0]*math.sin(pitch)
        My = M[0,0]*math.sin(pitch)*math.sin(roll)+M[1,0]*math.cos(roll)-M[2,0]*math.sin(roll)*math.cos(pitch)
        self.yaw = math.atan2(My,Mx)
    def Gyroscope(self,G:np.ndarray,delta):
        self.Groll = G[0,0]*delta
        self.Gpitch = G[1,0]*delta
        self.Gyaw = G[2,0]*delta

    def ComplementaryFilter(self):
        #Para el filtro complementario se utiliza para fusionar los sensores mediante una suma ponderada
        #Este filtro se aprovecha del hecho que el acelerometro (y magnetometro) es mas preciso cuando se hacen medidas
        # de baja frecuencia, pues no tiene un desfase ya que en forma estatica la aceleracion siempre es igual
        # Y en el caso del giroscopio, este es mas preciso en medidas de alte frecuencia, pero debido a que se 
        # necesita integrar para actualizar los angulos, existe un error que se acumula con el tiempo, por lo que no es
        # idoneo en medidas de baja frecuencia
        # El filtro complementario calcula un angulo de forma iterativa:
        # CFAngle = HP*(CFAngle*gyro*delta) + LP*acc
        # HP + LP = 1
        self.CF_LPWeigth = 1 - self.CF_HPWeigth
        self.CFAngle[0,0] = self.CF_HPWeigth * (self.CFAngle[0,0] * self.Groll) + self.CF_LPWeigth * self.roll
        self.CFAngle[1,0] = self.CF_HPWeigth * (self.CFAngle[1,0] * self.Gpitch) + self.CF_LPWeigth * self.pitch
        self.CFAngle[2,0] = self.CF_HPWeigth * (self.CFAngle[2,0] * self.Gyaw) + self.CF_LPWeigth * self.yaw
    def KalmanFilter(self):
        self.KFilterPitch.setAngle(self.pitch)
        self.KFilterRoll.setAngle(self.roll)
        self.KFilterYaw.setAngle(self.yaw)

#Implementacion basada en la implementacion del repositorio:
# https://github.com/TKJElectronics/KalmanFilter
class Kalman: