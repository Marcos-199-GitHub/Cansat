import numpy as np
import math
import calibracion
import serial
import time
import quaternion_filter
#Fuentes:

# 1.- Data Fusion with 9 Degrees of Freedom Inertial Measurement Unit To Determine Object’s Orientation
# By
# Long Tran
# Senior Project
# Electrical Engineering Department
# California Polytechnic State University San Luis Obispo June 2017

# 2.- https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html

RAD2DEG = 180/math.pi
DEG2RAD = math.pi/180
magnetic_declination = -7.51;  #// Japan, 24th June

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
        self.degrees = True
        
    #Obtiene los valores de roll y pitch mediante los valores de aceleracion (ya deberian estar calibrados)
    def Accelerometer(self,A:np.ndarray):
        # roll = atan(ay/az)
        self.roll = math.atan2(-A[1,0], A[2,0])
        # pitch = atan(-ax/sqrt(ay**2+az**2))
        # math.hypot is hypotenuse
        ##self.pitch = math.atan2(-A[0,0],math.hypot(A[1,0],A[2,0]))
        self.pitch = math.asin(A[0,0]/math.hypot(A[1,0],A[2,0],A[0,0]))
        if (self.degrees):
            self.pitch *= RAD2DEG
            self.roll *= RAD2DEG
    def Magnetometer(self,M:np.ndarray,pitch,roll):
        Mx = M[0,0]*math.cos(pitch) + M[2,0]*math.sin(pitch)
        My = M[0,0]*math.sin(pitch)*math.sin(roll)+M[1,0]*math.cos(roll)-M[2,0]*math.sin(roll)*math.cos(pitch)
        self.yaw = math.atan2(My,Mx)
        if (self.degrees):
            self.yaw *= RAD2DEG
    def Gyroscope(self,G:np.ndarray,delta):
        self.Groll = G[0,0]*delta
        self.Gpitch = G[1,0]*delta
        self.Gyaw = G[2,0]*delta
        if (not self.degrees):
            self.Gpitch *= DEG2RAD
            self.Groll *= DEG2RAD
            self.Gyaw *= DEG2RAD

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
    

def filter():
    mpu = serial.Serial(baudrate=38400,port="COM4",bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
    MPU = calibracion.MPU(mpu)
    F = Fusion()
    KYaw = Kalman()
    KPitch = Kalman()
    KRoll = Kalman()

    #Actualizar datos
    MPU.update()
    #Convertirlos a yaw, pitch, roll
    F.Accelerometer(MPU.acceleration)
    # F.Gyroscope(MPU.angularVelocity)
    #Inicializar el filtro Kalman con el pitch y roll del acelerometro
    KPitch.setAngle(F.pitch)
    KRoll.setAngle(F.roll)
    tStart = time.time();
    while True:
        #Actualizar datos
        MPU.update()
        #Convertirlos a yaw, pitch, roll
        F.Accelerometer(MPU.acceleration)
        # F.Gyroscope(MPU.angularVelocity)
        tStop = time.time()
        tElapse = tStop - tStart;
        tStart = time.time();
        temp = tElapse
        KPitch.getAngle(F.pitch,MPU._angularVelocityDeg[2,0],temp)
        KRoll.getAngle(F.roll,MPU._angularVelocityDeg[0,0],temp)

        print (f"A_pitch: {F.pitch:.2f}, A_Roll: {F.roll:.2f}....Pitch: {KPitch.angle:.2f}, Roll: {KRoll.angle:.2f}, dt: {temp:.4f}")

def update_rpy(qw: float, qx: float, qy: float, qz: float):
        # Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
        # In this coordinate system, the positive z-axis is down toward Earth.
        # Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
        # Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
        # Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
        # These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
        # Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
        # applied in the correct order which for this configuration is yaw, pitch, and then roll.
        # For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
        #
        # float a12, a22, a31, a32, a33;  // rotation matrix coefficients for Euler angles and gravity components
        rpy = np.array([[0],[0],[0]])
        a12 = 2.0 * (qx * qy + qw * qz)
        a22 = qw * qw + qx * qx - qy * qy - qz * qz
        a31 = 2.0 * (qw * qx + qy * qz)
        a32 = 2.0 * (qx * qz - qw * qy)
        a33 = qw * qw - qx * qx - qy * qy + qz * qz
        rpy[0,0] = math.atan2(a31, a33)
        rpy[1,0] = -math.asin(a32)
        rpy[2,0] = math.atan2(a12, a22)
        rpy[0,0] *= 180.0 / math.pi
        rpy[1,0] *= 180.0 / math.pi
        rpy[2,0] *= 180.0 / math.pi
        rpy[2,0] += magnetic_declination
        if (rpy[2,0] >= +180.0):
            rpy[2,0] -= 360.0
        elif (rpy[2,0] < -180.0):
            rpy[2,0] += 360.0
        
        return rpy

        # lin_acc[0] = a[0] + a31;
        # lin_acc[1] = a[1] + a32;
        # lin_acc[2] = a[2] - a33;

def updateQuaternion(QF:quaternion_filter.QuaternionFilter,A:np.ndarray,G:np.ndarray,M:np.ndarray,Q:np.ndarray,iteration:int) -> np.ndarray:
    # Madgwick function needs to be fed North, East, and Down direction like
    # (AN, AE, AD, GN, GE, GD, MN, ME, MD)
    # Accel and Gyro direction is Right-Hand, X-Forward, Z-Up
    # Magneto direction is Right-Hand, Y-Forward, Z-Down
    # So to adopt to the general Aircraft coordinate system (Right-Hand, X-Forward, Z-Down),
    # we need to feed (ax, -ay, -az, gx, -gy, -gz, my, -mx, mz)
    # but we pass (-ax, ay, az, gx, -gy, -gz, my, -mx, mz)
    # because gravity is by convention positive down, we need to ivnert the accel data
    # get quaternion based on aircraft coordinate (Right-Hand, X-Forward, Z-Down)
    # acc[mg], gyro[deg/s], mag [mG]
    # gyro will be convert from [deg/s] to [rad/s] inside of this function
    # quat_filter.update(-a[0], a[1], a[2], g[0] * DEG_TO_RAD, -g[1] * DEG_TO_RAD, -g[2] * DEG_TO_RAD, m[1], -m[0], m[2], q);
    an = -A[0,0]
    ae = +A[1,0]
    ad = +A[2,0]
    gn = +G[0,0] * DEG2RAD
    ge = -G[1,0] * DEG2RAD
    gd = -G[2,0] * DEG2RAD
    mn = +M[1,0]
    me = -M[0,0]
    md = +M[2,0]
    for i in range(0,iteration):
        Q = QF.update(an, ae, ad, gn, ge, gd, mn, me, md, Q)
    return Q


def filter2():
    mpu = serial.Serial(baudrate=38400,port="COM4",bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
    MPU = calibracion.MPU(mpu)
    

    #Actualizar datos
    MPU.update()
    Quaternion = quaternion_filter.QuaternionFilter()
    Quaternion.filter_sel = "NO"
    #Temporal
    magneticField = np.array([[1],[0],[0]])
    Q = np.array([1,1,1,1])
    D = np.array([[0],[0],[0]])
    while True:
        #Actualizar datos
        MPU.update()
        Q = updateQuaternion(Quaternion,MPU._accelerationG,MPU._angularVelocityDeg,magneticField,Q,1)
        D = update_rpy(Q[0], Q[1], Q[2], Q[3])
        print (f"Roll: {D[0,0]}, Pitch: {D[1,0]}, Yaw: {D[2,0]}")
        # Q = Quaternion.update(MPU._accelerationG[0,0],MPU._accelerationG[0,1],MPU._accelerationG[0,2],
                        #   MPU._angularVelocityDeg[0,0],MPU._angularVelocityDeg[0,1],MPU._angularVelocityDeg[0,2],
                        #   magneticField[0,0],magneticField[0,1],magneticField[0,2],Q)


if __name__ == "__main__":
    filter()

