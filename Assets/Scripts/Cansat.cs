using System;
using System.Collections;
using System.Collections.Generic;
//using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Runtime.InteropServices.ComTypes;
using System.Threading;
using UnityEditor.VersionControl;
using UnityEngine;
using UnityEngine.UI;

public class Cansat : MonoBehaviour{

    private static SerialPort _serialPort;
    Thread                    readThread = new Thread( Read );
    private bool              _isConnected;

    public static  MPU6050      mpu6050 = new MPU6050();
    public static  BMP280       bmp280  = new BMP280();
    const          string       PATH    = "Assets/registro.txt";
    private static StreamReader reader;

    public static DataRecived dataRecived = new DataRecived();

    public static bool rotar = false;

    private static float tiempo;
    public string puerto = "COM5";
    public int baud = 57600;

    private static int imgLength = 0;
    private static bool isImage = false;
    private static Texture2D receivedImageTexture;
    public Image imagen;
    public static byte[] imageBytes;
    public static bool imageUpdated = false;


    private void Start(){
        receivedImageTexture = new Texture2D(10, 10);
        reader                  = new StreamReader( PATH );
        _serialPort             = new SerialPort( puerto, baud );
        _serialPort.ReadTimeout = 500;
        readThread.Start();
    }

    private void Update(){
        tiempo = Time.time;
        if( rotar ){
            transform.Rotate( dataRecived.getDeltaAngle() );
            rotar = false;
        }
        //transform.position += mpu6050.getDeltaPosition();

        if (imageUpdated)
        {
            imageUpdated = false;
            DataRecived.updateImage();
            Sprite sprite = Sprite.Create(
                receivedImageTexture, new Rect(0, 0, receivedImageTexture.width, receivedImageTexture.height), Vector2.one * 0.5f
            );

            imagen.sprite = sprite;
        }
    }

    private void OnDisable(){
        readThread.Abort();
        _serialPort.Close();
        reader.Close();
    }

    private static int id = 0;

    public static string getDataString( int method = 0 ){
        string message = "";
        if( id == 350 ){
            Debug.Log( "ya" );
            return "";
        }

        if( method == 0 ){             //Serial
            if( !_serialPort.IsOpen ){ //comprobamos que el puerto esta abierto
                Debug.Log( "Serial is closed" );
                try{
                    Debug.Log( "Tratando de abrir puerto serial" );
                    _serialPort.Open();
                    Debug.Log( "Listo" );
                }
                catch( Exception e ){
                    Debug.Log( "Error" );
                    Debug.Log( e );
                }

                return "";
            }

            message = _serialPort.ReadTo( "\n" );
        }
        else if( method == 1 ){ //Archivo
            message = reader.ReadLine();
            id++;
            Debug.Log( id );
        }

        

        //Debug.Log( message );
        return message;
    }

    public static byte[] getDataImg(int method = 0)
    {

        byte[] imageData = new byte[imgLength];
            if (method == 0)
            {             //Serial
            if (!_serialPort.IsOpen)
            { //comprobamos que el puerto esta abierto
                Debug.Log("Serial is closed");
                try
                {
                    Debug.Log("Tratando de abrir puerto serial");
                    _serialPort.Open();
                    Debug.Log("Listo");
                }
                catch (Exception e)
                {
                    Debug.Log("Error");
                    Debug.Log(e);
                }

            }
            else
            {
                Debug.Log("Leyendo");
                try
                {
                    int x = 0;
                    int maxIter = 1000;
                    int iter;
                    for (int i=0;i<imgLength;i++)
                    {
                        iter = 0;

                        do { 
                            x = _serialPort.ReadByte();
                            iter++;
                            if (iter >= maxIter) Debug.LogError("Excedido el tiempo de espera para el serial !!");
                        }
                        while (x == -1);

                        imageData[i] = (byte)x;

                    }
                    Debug.Log(String.Format("Buffer de {0} bytes leido",imageData.Length));
                    Debug.Log(BitConverter.ToString(imageData));

                }
                catch (Exception e)
                {
                    Debug.LogException(e);
                }
            }
        }
        return imageData;
        
    }
    private static float tprevious; //s

    public static void Read(){
        Debug.Log( "Thread de lectura iniciado" );
        while( true ){
            try{
                if (!isImage) dataRecived.updateData(getDataString());
                //else dataRecived.updateImage(getDataImg());
                else { 
                    imageBytes = getDataImg();
                    imageUpdated = true;
                    Debug.Log("Recibida una imagen");
                    isImage = false;
                }
                rotar = true;
            }
            catch( TimeoutException ){
            }
        }
        
    }

    public class DataRecived{

        public  float[] T = new float[3];
        public  float   Pressure;
        public  float   Altitude;
        public  float   Time;
        public  float   Humidity;
        public  Vector3 Acc       = new Vector3();
        public  Vector3 Gyro      = new Vector3();
        //Nuevos
        public  Vector3 Magnet    = new Vector3();
        public float Heading = 0f;
        public float Yaw = 0f;
        public float Pitch = 0f;
        public float Roll = 0f;
        public float CansatDt = 0f;
        public float Latitud = 0f;
        public float Longitud = 0f;
        public float Speed = 0f;
        public float PDOP = 0f;

        private float   PrevTime  = 0.0f;
        public  float   DeltaTime = 0.05f;

        public string rawMessage = "";

        private void restart(){
            T[0]     = 0;
            T[1]     = 0;
            T[2]     = 0;
            Pressure = 0;
            Altitude = 0;
            Time     = 0;
            Humidity = 0;
            Acc      = new Vector3();
            Gyro     = new Vector3();
            Magnet   = new Vector3();
            Heading  = 0f;
            Yaw      = 0f;
            Pitch    = 0f;
            Roll     = 0f;
            CansatDt = 0f;
            Latitud  = 0f;
            Longitud = 0f;
            Speed    = 0f;
            PDOP     = 0f;
    }

        public void updateData( string message ){
            //Debug.Log( message );
            if( !message.StartsWith( "{" ) ){
                Debug.Log( "Dato incorrecto, incompleto" );
                Debug.Log( message );
                restart();
                return;
            }
            rawMessage = message;
            message = message.Substring( 1, message.Length - 2 ).Replace( " ", "" );//Quitar las llaves y salto de linea
            string[] data = message.Split( ',' ); 

            //Estaba mejor la otra implementacion, porque ocupa menos ancho de banda y asi recibe mensajes mas rapido
            //Sorry :')
            //Formato: 

            /*
             {T2,T3,T,A,P,H,Ax,Ay,Az,Wx,Wy,Wz,Mx,My,Mz,He,Y,P,R,Dt,Lt,Lg,Km,DP,Im}

            0,T2 temperatura del sht °C
            1,T3 temperatura del mpu °C
            2,P Presion hPa
            3,A Altitud m
            4,T Tiempo (en segundos con respecto al EPOCH )s
            5,H Humedad relativa %
            6,Ax AccelX m/s2
            7,Ay AccelY m/s2
            8,Az AccelZ m/s2
            9,Wx GyroX °/s
            10,Wy GyroY °/s
            11,Wz GyroZ °/s
            12,Mx MagnetX uT
            13,My MagnetY uT
            14,Mz MagnetZ uT
            15,He Direccion a la que apunta la brujula °
            16,Y  Yaw °
            17,P  Pitch °
            18,R  Roll °
            19,Dt Delta T del filtro s
            20,Lt Latitud °
            21,Lg Longitud °
            22,Km Velocidad respecto al suelo Km/h
            23,DP Dilucion de precision del GPS
            24,Im Si el proximo mensaje va a ser una imagen, su valor es el tamaño en bytes, si no, es 0
             
             */


            //T[0] = 0;

            float _x, _y, _z;
            int b = 0;

            try
            {
                float.TryParse(data[0], out T[1]);
                float.TryParse(data[1], out T[2]);
                float.TryParse(data[2], out Time);
                float.TryParse(data[3], out Pressure);
                float.TryParse(data[4], out Humidity);
                float.TryParse(data[5], out Heading);

                float.TryParse(data[6], out _x);
                float.TryParse(data[7], out _y);
                float.TryParse(data[8], out _z);
                Acc = new Vector3(_x, _y, _z);
                float.TryParse(data[9], out _x);
                float.TryParse(data[10], out _y);
                float.TryParse(data[11], out _z);
                Gyro = new Vector3(_x, _y, _z);
                float.TryParse(data[12], out _x);
                float.TryParse(data[13], out _y);
                float.TryParse(data[14], out _z);
                Magnet = new Vector3(_x, _y, _z);


                float.TryParse(data[16], out Yaw);
                float.TryParse(data[17], out Pitch);
                float.TryParse(data[18], out Roll);
                float.TryParse(data[19], out CansatDt);
                float.TryParse(data[20], out Latitud);
                float.TryParse(data[21], out Longitud);
                float.TryParse(data[22], out Speed);
                float.TryParse(data[23], out PDOP);

                int.TryParse(data[24], out b);
            }
            catch (Exception e)
            {
                Debug.Log("Dato perdido, error leyendo");
            }

            if (b > 0)
            {
                Debug.Log(rawMessage);
                Debug.Log(String.Format("Imagen de {0} bytes", b));
                isImage = true;
                imgLength = b;
            }


        }

        public static void updateImage()
        {
            //Convertir de JPG a textura normal
            //System.Drawing.Image x = (Bitmap)((new ImageConverter()).ConvertFrom(imageBytes));
            //receivedImageTexture = new Texture2D()
            //ImageConversion.LoadImage(receivedImageTexture, imageBytes);
            string imageName = String.Format("{0}.jpeg",System.DateTime.Now.Ticks);
            File.WriteAllBytes(imageName, imageBytes);
            receivedImageTexture.LoadRawTextureData(imageBytes);
            receivedImageTexture.Apply();
        }
        public Vector3 getDeltaAngle(){
            return new Vector3(
                Gyro.x * DeltaTime,
                0,
                Gyro.y * DeltaTime
            );
        }
    }

    public class MPU6050{
        public  Vector3 Acc;           //m/s^2
        public  Vector3 Gyro;          //grad/s
        private int     acc_index = 6; //Indice del json donde empiezan los datos
        public  float   last_time = 0.0f;

        public void update( string message ){
            if( !message.StartsWith( "{" ) ){
                Debug.Log( "Dato incorrecto, incompleto" );
                return;
            }

            message = message.Substring( 1, message.Length - 2 );
            //Debug.Log(message);
            string[] data = message.Split( ',' ); //"T1:123215", "T2:126843"
            //Debug.Log(data);
            if( data.Length < 6 ){ //Datos incompletos
                return;
            }

            //El indice 4 es el del tiempo
            last_time = float.Parse(data[2] );
            Acc = new Vector3( float.Parse( data[acc_index] ), float.Parse( data[acc_index + 1] ), float.Parse( data[acc_index + 2] ) );
            Gyro = new Vector3(
                float.Parse( data[acc_index + 3] ), float.Parse( data[acc_index + 4] ), float.Parse( data[acc_index + 5] )
            );
        }

        public Vector3 getDeltaAngle( float deltaTime = 0.1f ){
            return new Vector3(
                Gyro.x * deltaTime,
                0,
                Gyro.y * deltaTime
            );
        }

        public Vector3 getDeltaPosition( float deltaTime = 0.1f ){
            return Acc * ( deltaTime * deltaTime ) / 2;
        }
    }

    public class BMP280{
        public float Temp;
        public float Pressure;

        public void update( string message ){
            string[] data = message.Split( ',' );
            Temp     = float.Parse( data[0] );
            Pressure = float.Parse( data[3] );
        }
    }

}