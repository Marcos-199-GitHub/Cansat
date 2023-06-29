import numpy as np
import serial
import math
import time

GRAVITY = np.array([[0],[0],[9.81]])

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

class Calibration:
    def __init__(self) -> None:
        self.V = np.zeros((3,3))
        self.B = np.zeros((3,1))
        pass
    def Accelerometer(self,U: np.ndarray):
        g = GRAVITY[2][0]
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
    def Accelerometer2(self,U: np.ndarray):
        #Este metodo tiene la ventaja de no requerir una matriz de referencia, solo que los vectores de orientacion
        # Generen una matriz a la que se le pueda calcular el inverso multiplicativo:
        g = GRAVITY[2][0]
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