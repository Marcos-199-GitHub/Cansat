import numpy as np
import serial
import math
import time

RAD2DEG = 180/math.pi
DEG2RAD = math.pi/180
GRAVITY = np.array([[0],[0],[9.81]])
g = GRAVITY[2][0]
LATITUDE = 45
Wie = 5
EARTH_ROTATION = np.array([[math.cos(LATITUDE*DEG2RAD)],[0],[math.sin(LATITUDE*DEG2RAD)]]) * Wie
mpu = serial.Serial(baudrate=38400,port="COM4",bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

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


#Fuentes:
# 1.- A Multi-Position Calibration Algorithm for Inertial Measurement Units. 
# Hongliang Zhang, Yuanxin Wu, Meiping Wu, Xiaoping Hu and Yabing Zha∗. 
# National University of Defense Technology, Changsha, Hunan, P. R. China
# 2.- Improved multi-position calibration for inertial measurement units
# Hongliang Zhang, YuanxinWu,WenqiWu,MeipingWu and Xiaoping Hu
# Department of Automatic Control, College of Mechatronics and Automation, National University of
# Defense Technology, Changsha, Hunan, 410073, People’s Republic of China
class Calibration:
    def __init__(self) -> None:
        self.V = np.zeros((3,3))
        self.B = np.zeros((3,1))
        pass
    # Calibrates Accelerometer, requires raw dato from accelerometer (6 orientations)
    def Accelerometer(self,U: np.ndarray):
        
        #Matriz de referencia con 6 vectores que representan orientaciones del acelerometro
        P = np.array( [
            [g ,-g , 0 , 0 , 0 , 0],
            [0 , 0 , g ,-g , 0 , 0],
            [0 , 0 , 0 , 0 , g ,-g], 
            [1 , 1 , 1 , 1 , 1 , 1],
        ] )

        #P = np.array( [[g ,-g , 0 , 0 , 0 , 0],[0 , 0 , g ,-g , 0 , 0],[0 , 0 , 0 , 0 , g ,-g], [1 , 1 , 1 , 1 , 1 , 1]] )
        #La matriz U debe contener mediciones reales de estas 6 orientaciones:
        #Medidas tomadas para calibrar (6 medidas) (Se pueden añadir mas siempre y
        # % cuando tambien se añadan a la matriz
        # % de referencia)
        # U = [g -g  0  0  0  0 
        #     0  0  g -g  0  0 
        #     0  0  0  0  g -g ]
        PPT = np.matmul(P,np.transpose(P))
        PPTI = np.linalg.inv(PPT)
        UPT = np.matmul(U,np.transpose(P))
        #Esta matriz contendra los componentes de escala y los bias
        W = np.matmul(UPT,PPTI)
        #Matriz con los componentes de escala
        self.V = np.array([[W[0][0] , W[1][0], W[2][0]],
                      [W[0][1] , W[1][1], W[2][1]],
                      [W[0][2] , W[1][2], W[2][2]]])
        #%Matriz con el BIAS
        self.B = np.array([[W[3][0]],
                      [W[3][1]],
                      [W[3][2]]])
        #% Para una medida cualquiera del sensor A
        #A = [1
        #     1
        #     1]
        # La medida corregida de A sera:
        # a = inv(V) * A + B
    #Calibrates Accelerometer, requires raw data from accelerometer (9 orientations)
    def Accelerometer2(self,U: np.ndarray):
        #Este metodo tiene la ventaja de no requerir una matriz de referencia, solo que los vectores de orientacion
        # Generen una matriz a la que se le pueda calcular el inverso multiplicativo:
        #La matriz U debe contener mediciones reales de estas 9 orientaciones:
        #Medidas tomadas para calibrar (9 medidas) 
        # U = [g -g  0  0  0  0  ...
        #     0  0  g -g  0  0  ...
        #     0  0  0  0  g -g ...]

        # Hay que definir un vector de longitud igual a la cantidad de datos de aceleracion en U
        nDatos = np.shape(U)[1]
        #Este vector estara compuesto unicamente de la magnitud G = 9.81 al cuadrado
        W = np.ones((nDatos,1))*g*g
        # Asumiendo que el error aleatorio del sensor ya fue tomado en cuenta haciendo un promedio
        #  de las medidas de cada orientacion, tenemos:
        # W = N.K.r
        # donde r , k es una matriz, y N es:
        N = np.zeros((nDatos,9))
        for i in range (0,nDatos):
            ax = U[0][i]
            ay = U[1][i]
            az = U[2][i]
            # Se define el sig vector:
            Ni = np.array([[ax**2, ay**2, az**2, ax*ay, ax*az, ay*az, -2*ax, -2*ay, -2*az]])
            # Se añade al vector N
            N[i] = Ni
        # A partir de esto, se puede encontrar K.r utilizando el metodo de minimos cuadrados:
        # Utilizando la ecuacion AT.A.x = AT.b y despejando x
        # x = (AT.b).inv(A.At)
        NTW = np.matmul(np.transpose(N),W)
        NNT = np.matmul(N,np.transpose(N))
        NNTI = np.linalg.inv(NNT)
        
        Kr = np.matmul(NNTI,NTW)
        print (Kr)
        # Una ver obtenido Kv'r, se pueden obtener sqrt(r)*Ka y sqrt(r)*fa
        #Asumiendo que los factores de escala Sx, Sy, Sz son siempre positivos
        lambda1 = 1
        lambda2 = 1
        lambda3 = 1
        srKa = np.zeros((3,3))
        #Fila 3
        srKa[2,2] = lambda3*math.sqrt(Kr[2,0])
        srKa[2,1] = Kr[5,0]/srKa[2,2]
        srKa[2,0] = Kr[4,0]/srKa[2,2]
        #Fila 2
        srKa[1,1] = lambda2*math.sqrt(Kr[1,0] - Kr[2,1]**2)
        srKa[1,0] = (Kr[3]-srKa[2,0]*srKa[2,1])/srKa[1,1]
        #Fila 3
        srKa[0,0] = lambda1*math.sqrt(Kr[0,0]-srKa[1,0]**2-srKa[2,0]**2)
        #########
        srF0 = np.zeros((3,1))
        srF0[2,0] = Kr[8,0]/srKa[2,2]
        srF0[1,0] = (Kr[7,0] - srKa[2,1]*srF0[2,0])/srKa[1,1]
        srF0[0,0] = (Kr[6,0]-srKa[1,0]*srF0[1,0]-srKa[2,0]*srF0[2,0])/srKa[0,0]
        # Con esto, utilizando la siguiente ecuacion, se puede despejar r
        # |g|**2 = (Ka.Na-F0)T.(Ka.Na-F0); multiplicando ambos lados por r y despejando:
        KNf = np.add(np.matmul(srKa, np.transpose([N[:,0]])) , -srF0)
        #Recordar que r es un escalar
        r = (np.matmul(KNf,np.transpose(KNf)) / (g**2))[0]
        #Con esto 
        Ka = srKa / math.sqrt(r)
        F0 = srF0 / math.sqrt(r)
        # Donde f0 corresponde al bias y Ka corresponde a la matriz de
        # no-ortogonalidad junto con los factores de escala
        # Es decir, para una aceleracion N, la aceleracion real F sera:
        # F = Ka.N - F0 - Err (Err es un error aleatorio que no se toma en cuenta pero existe)
        self.V = Ka
        self.B = F0
    #Calibrates gyroscope, requires calibrated dato from accelerometer (F) an raw data from gyro (Ng) (12-18 orientations each)
    def Gyroscope(self, Ng:np.ndarray,F:np.ndarray):
        # En el caso del giroscopio, se necesita que N y F contengan 12 orientaciones diferentes
        # Hay que definir un vector de longitud igual a la cantidad de datos de aceleracion en Ng
        nDatos = np.shape(Ng)[1]        #Este vector estara compuesto unicamente de la magnitud Wie*G*Sin(Latitud)
        W = np.ones((nDatos,1))*Wie*g*math.sin(LATITUDE)
        N = np.zeros((nDatos,12))
        for i in range (0,nDatos):
            ax = F[0][i]
            ay = F[1][i]
            az = F[2][i]

            wx = Ng[0][i]
            wy = Ng[1][i]
            wz = Ng[2][i]
            # Se define el sig vector:
            Ni = np.array([[ax*wx,ax*wy,ax*wz, ay*wx,ay*wy,ay*wz, az*wx,az*wy,az*wz, -ax,-ay,-az]])
            # Se añade al vector N
            N[i] = Ni
        #Usando el metodo de minimos cuadrados:
        # Kv = (NT.W).inv(NT.N)
        NTW = np.matmul(np.transpose(N),W)
        NTN = np.matmul(np.transpose(N),N)
        NTNI = np.linalg.inv(NTN)
        Kv = np.matmul(NTW,NTNI)
        #Kv = [K11,k12,k13,k21,k22,k23,k31,k32,k33,wx,wy,wz]
        Kg = np.array([[Kv[0,0],Kv[0,1],Kv[0,2]],
                       [Kv[0,3],Kv[0,4],Kv[0,5]],
                       [Kv[0,6],Kv[0,7],Kv[0,8]]])
        W0 = np.array([[Kv[0,9]],
                       [Kv[0,10]],
                       [Kv[0,11]]])
        self.V = Kg
        self.B = W0
    #Returns corrected value after calibration
    def real(self,U:np.ndarray):
        corrected = np.matmul(self.V,U) - self.B
        return corrected   





# g = GRAVITY[2][0]
# #U debe contener 9 orientaciones diferentes
# U = np.array([[g,-g,0,0,0,0],
#               [0,0,g,-g,0,0],
#               [0,0,0,0,g,-g]]) 

#Primero hay que tomar 9 medidas en 9 diferentes orientaciones (H Zhang et al)
def Orientaciones(stillTime):
    M = np.zeros((3,9))
    for i in range (0,9):
        print (f"Realiza la orientacion {i+1} y mantente ahi por {stillTime} segundos")
        input("Presiona enter para comenzar a medir")
        #Limpiar el buffer del puerto serial
        mpu.read_all()
        print ("Comenzando medicion")
        n = 0
        start = time.time()
        A = np.zeros((3,1))
        while ((time.time() - start) < stillTime):
            try:
                D.update()
            except:
                continue
            A = np.append(A,D.acceleration,axis=1)
            n+=1
        #Quitar el primer vector que se uso para inicializar el array [[0],[0],[0]]]
        A = A[:,1:]
        #Hacer el promedio de las mediciones
        MeanA = np.mean(A,axis=1)
        #Colocarlo en la matriz final
        M[:,i] = MeanA
        print (f"El promedio de las {n} mediciones fue {MeanA}")
    return M

U = Orientaciones(2)
C = Calibration()   
C.Accelerometer2(U) 


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
        self.CFAngle = np.zeros((3,1))
        #Complementary filter High pass weigth
        self.CF_HPWeigth = 0
        #Complementary filter Low pass weigth
        #DONT USE: it will be overriden by HPWeigth
        self.CF_LPWeigth = 0
        #HP + LP = 1
        
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
    def getAngle(self):
        return self.angle
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
        rate = newRate - bias
        angle += dt * rate
        #  Update estimation error covariance - Project the error covariance ahead
        #  Step 2
        P[0,0] += dt * (dt*P[1,1] - P[0,1] - P[1,0] + Q_angle)
        P[0,1] -= dt * P[1,1]
        P[1,0] -= dt * P[1,1]
        P[1,1] += Q_bias * dt
        # Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        # Calculate Kalman gain - Compute the Kalman gain
        # Step 4
        S = P[0][0] + R_measure # // Estimate error
        # Step 5
        K = np.zeros((2,1)) #// Kalman gain - This is a 2x1 vector
        K[0,0] = P[0,0] / S
        K[1,0] = P[1,0] / S
        # Calculate angle and bias - Update estimate with measurement zk (newAngle)
        # Step 3
        y = newAngle - angle # Angle difference
        # Step 6
        angle += K[0,0] * y
        bias += K[1,0] * y

        # Calculate estimation error covariance - Update the error covariance
        # Step 7
        P00_temp = P[0,0]
        P01_temp = P[0,1]

        P[0,0] -= K[0,0] * P00_temp
        P[0,1] -= K[0,0] * P01_temp
        P[1,0] -= K[1,0] * P00_temp
        P[1,1] -= K[1,0] * P01_temp

        return angle

