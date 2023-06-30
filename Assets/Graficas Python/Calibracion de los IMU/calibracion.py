import numpy as np
import serial
import math
import time
import fusion

RAD2DEG = 180/math.pi
DEG2RAD = math.pi/180
GRAVITY = np.array([[0],[0],[9.81]])
g = GRAVITY[2][0]
# En donde estoy la Latitud es 19°50'0'' = 19° + 50'/60 + 0''/3600 
# = 19.833°
LATITUDE = 19.833 
# Wie = 15 deg/h =  0.0041666666666 deg/s
Wie = 0.00416666 # deg/s
EARTH_ROTATION = np.array([[math.cos(LATITUDE*DEG2RAD)],[0],[math.sin(LATITUDE*DEG2RAD)]]) * Wie
mpu = serial.Serial(baudrate=38400,port="COM4",bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
ACC_ORIENTATION_SAVE_FILE = "calibration_data/Acc_orientation.npy"
GYRO_ORIENTATION_SAVE_FILE = "calibration_data/Gyro_orientation.npy"
GYRO_CALIBRATION_SAVE_FILE = "calibration_data/Gyro_calibration.npy"
ACC_CALIBRATION_SAVE_FILE = "calibration_data/Acc_calibration.npy"
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
    def clean(self):
        self.mpu.read_all()
        self.all_data = ""
        self.ended = False
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
        self.SaveMat = np.zeros((3,4))
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
        NTN = np.matmul(np.transpose(N),N)
        NTNI = np.linalg.inv(NTN)
        
        Kr = np.matmul(NTNI,NTW)
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
        srKa[1,1] = lambda2*math.sqrt(Kr[1,0] - Kr[2,0]**2)
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
        NiT = np.transpose([U[:,0]])
        KNf = np.add(np.matmul(srKa,NiT ) , -srF0)
        #Recordar que r es un escalar
        r = (np.matmul(np.transpose(KNf),KNf) / (g**2))
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
    #Saves calibration data on a file
    def saveCalibration(self,isGyro = False):
        for i in range(0,3):
            for j in range (0,3):
                self.SaveMat[i,j] = self.V[i,j]
        for i in range (0,3):
            self.SaveMat[i,3] = self.B[i,0]
        if (isGyro):
            with open(GYRO_CALIBRATION_SAVE_FILE,'wb') as f:
                np.save(f,self.SaveMat)
        else:
            with open(ACC_CALIBRATION_SAVE_FILE,'wb') as f:
                np.save(f,self.SaveMat)
    #Loads calibration data from file
    def loadCalibration(self, isGyro = False):
        if (isGyro):
            with open(GYRO_CALIBRATION_SAVE_FILE,'rb') as f:
                self.SaveMat = np.load(f)
        else:
            with open(ACC_CALIBRATION_SAVE_FILE,'rb') as f:
                self.SaveMat = np.load(f)
        for i in range(0,3):
            for j in range (0,3):
                self.V[i,j] = self.SaveMat[i,j]
        for i in range (0,3):
            self.B[i,0] = self.SaveMat[i,3]
        

# g = GRAVITY[2][0]
# #U debe contener 9 orientaciones diferentes
# U = np.array([[g,-g,0,0,0,0],
#               [0,0,g,-g,0,0],
#               [0,0,0,0,g,-g]]) 

#Primero hay que tomar 9 medidas en 9 diferentes orientaciones (H Zhang et al)
def GetAccOrientations(sensor: MPU,stillTime, save = True):
    print (f"Orientacion del acelerometro")
    M = np.zeros((3,9))
    for i in range (0,9):
        print (f"Realiza la orientacion {i+1} y mantente ahi por {stillTime} segundos")
        input("Presiona enter para comenzar a medir")
        #Limpiar el buffer del puerto serial
        sensor.clean()
        print ("Comenzando medicion")
        n = 0
        start = time.time()
        A = np.zeros((3,1))
        while ((time.time() - start) < stillTime):
            try:
                sensor.update()
            except:
                continue
            A = np.append(A,sensor.acceleration,axis=1)
            n+=1
        #Quitar el primer vector que se uso para inicializar el array [[0],[0],[0]]]
        A = A[:,1:]
        #Hacer el promedio de las mediciones
        MeanA = np.mean(A,axis=1)
        #Colocarlo en la matriz final
        M[:,i] = MeanA
        print (f"El promedio de las {n} mediciones fue {MeanA}")
    if (save):
        with open(ACC_ORIENTATION_SAVE_FILE, 'wb') as f:
            np.save(f,M)
    return M
#Primero hay que tomar 12 medidas en 12 diferentes orientaciones (H Zhang et al)
def GetGyroOrientations(sensor: MPU,stillTime, save = True):
    print (f"Orientacion del giroscopio")
    M = np.zeros((3,12))
    for i in range (0,12):
        print (f"Realiza la orientacion {i+1} y mantente ahi por {stillTime} segundos")
        input("Presiona enter para comenzar a medir")
        #Limpiar el buffer del puerto serial
        sensor.clean()
        print ("Comenzando medicion")
        n = 0
        start = time.time()
        A = np.zeros((3,1))
        while ((time.time() - start) < stillTime):
            try:
                sensor.update()
            except:
                continue
            A = np.append(A,sensor.angularVelocity,axis=1)
            n+=1
        #Quitar el primer vector que se uso para inicializar el array [[0],[0],[0]]]
        A = A[:,1:]
        #Hacer el promedio de las mediciones
        MeanA = np.mean(A,axis=1)
        #Colocarlo en la matriz final
        M[:,i] = MeanA
        print (f"El promedio de las {n} mediciones fue {MeanA}")
    if (save):
        with open(GYRO_ORIENTATION_SAVE_FILE, 'wb') as f:
            np.save(f,M)
    return M

if __name__ == "__main__":
    A = None
    with open("calibration_data/Acc_orientation.npy", 'rb') as f:
        A = np.load(f)
    #U = GetAccOrientations(D,2,save=True)
    CalibrationAcc = Calibration()   
    CalibrationAcc.Accelerometer2(A)
    CalibrationAcc.saveCalibration(isGyro=False)

    # U = GetGyroOrientations(D,2,save=True)
    # CalibrationGyro = Calibration()   
    # CalibrationGyro.Gyroscope(U)
    # CalibrationGyro.saveCalibration(isGyro=True)