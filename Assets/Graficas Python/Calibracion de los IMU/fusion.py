import numpy as np
import math
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
        self.pitch = math.atan2(-A[0,0],math.hypot(A[1,0],A[2,0]))
    def Magnetometer(self,M:np.ndarray,pitch,roll):
        Mx = M[0,0]*math.cos(pitch) + M[2,0]*math.sin(pitch)
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
    def __init__(self) -> None:
        #  We will set the variables like so, these can also be tuned by the user 
        self.Q_angle = 0.001
        self.Q_bias = 0.003
        self.R_measure = 0.03

        self.angle = 0.0 #// Reset the angle
        self.bias = 0.0 #// Reset bias
        self.rate = 0.0
        self.P = np.zeros((2,2))
         #// Since we assume that the bias is 0 and we know the starting angle (use setAngle), 
         # the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
        self.P[0,0] = 0.0
        self.P[0,1] = 0.0
        self.P[1,0] = 0.0
        self.P[1,1] = 0.0
    def getRate(self):
        return self.rate
    def setAngle(self,angle):
        self.angle = angle
    def setQangle(self,Qangle):
        self.Q_angle = Qangle
    def getQangle(self):
        return self.Q_angle
    def setQbias(self,Qbias):
        self.Q_bias = Qbias
    def getQbias(self):
        return self.Q_bias
    def setRmeasure(self,Rmeasure):
        self.R_measure = Rmeasure
    def getRmeasure(self):
        return self.R_measure
    #TODO: asegurarse que la notacion de c++ P[m][n] es equivalente a la de numpy P[m,n] 
    # The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    def getAngle(self, newAngle,newRate,dt):
        # KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        # Modified by Kristian Lauszus
        # See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
        # Discrete Kalman filter time update equations - Time Update ("Predict")
        # Update xhat - Project the state ahead
        # Step 1
        self.rate = newRate - self.bias
        self.angle += dt * self.rate
        #  Update estimation error covariance - Project the error covariance ahead
        #  Step 2
        self.P[0,0] += dt * (dt*self.P[1,1] - self.P[0,1] - self.P[1,0] + self.Q_angle)
        self.P[0,1] -= dt * self.P[1,1]
        self.P[1,0] -= dt * self.P[1,1]
        self.P[1,1] += self.Q_bias * dt
        # Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        # Calculate Kalman gain - Compute the Kalman gain
        # Step 4
        S = self.P[0][0] + self.R_measure # // Estimate error
        # Step 5
        K = np.zeros((2,1)) #// Kalman gain - This is a 2x1 vector
        K[0,0] = self.P[0,0] / S
        K[1,0] = self.P[1,0] / S
        # Calculate angle and bias - Update estimate with measurement zk (newAngle)
        # Step 3
        y = newAngle - self.angle # Angle difference
        # Step 6
        self.angle += K[0,0] * y
        self.bias += K[1,0] * y

        # Calculate estimation error covariance - Update the error covariance
        # Step 7
        P00_temp = self.P[0,0]
        P01_temp = self.P[0,1]

        self.P[0,0] -= K[0,0] * P00_temp
        self.P[0,1] -= K[0,0] * P01_temp
        self.P[1,0] -= K[1,0] * P00_temp
        self.P[1,1] -= K[1,0] * P01_temp

        return self.angle