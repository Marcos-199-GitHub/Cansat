using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Threading;
using UnityEditor.VersionControl;
using UnityEngine;

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

    private void Start(){
        reader                  = new StreamReader( PATH );
        _serialPort             = new SerialPort( ".\\COM4", 9600 );
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
        }

        id++;
        Debug.Log( id );

        //Debug.Log( message );
        return message;
    }

    private static float tprevious; //s

    public static void Read(){
        Debug.Log( "Thread de lectura iniciado" );
        while( true ){
            //delay 100ms
            if( tiempo - tprevious < 0.333f )
                continue;

            tprevious = tiempo;
            //Debug.Log("Leyendo");
            try{
                dataRecived.updateData( getDataString( 1 ) );
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
        private float   PrevTime  = 0.0f;
        public  float   DeltaTime = 0.1f;

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
        }

        public void updateData( string message ){
            if( !message.StartsWith( "{" ) ){
                Debug.Log( "Dato incorrecto, incompleto" );
                restart();
                return;
            }

            message = message.Substring( 1, message.Length - 2 );
            string[] data = message.Split( ',' ); //" 'T1': '123215'", " 'T2': '126843'"

            for( int i = 0; i < data.Length; i++ ){
                string[] current = data[i].Split( ':' );                             //"'T1'", "'123215'"
                string   key     = current[0].Substring( 2, current[0].Length - 3 ); //Quitar el espacio y las comillas
                string   val     = current[1].Substring( 2, current[1].Length - 3 ); //Quitar el espacio y las comillas
                //Debug.Log( key + ", " + val );
                float valor = float.Parse( val );

                switch( key ){
                    //Keys: T1, T2, T3, P, A, T, H, Ax, Ay, Az, Wx, Wy, Wz
                    case "1":
                        T[0] = valor;
                        break;
                    case "T2":
                        T[1] = valor;
                        break;
                    case "T3":
                        T[2] = valor;
                        break;
                    case "P":
                        Pressure = valor;
                        break;
                    case "A":
                        Altitude = valor;
                        break;
                    case "T":
                        PrevTime  = Time;
                        Time      = valor;
                        DeltaTime = Time - PrevTime;
                        break;
                    case "H":
                        Humidity = valor;
                        break;
                    case "Ax":
                        Acc.x = valor;
                        break;
                    case "Ay":
                        Acc.y = valor;
                        break;
                    case "Az":
                        Acc.z = valor;
                        break;
                    case "Wx":
                        Gyro.x = valor;
                        break;
                    case "Wy":
                        Gyro.y = valor;
                        break;
                    case "Wz":
                        Gyro.z = valor;
                        break;
                }
            }
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
        private int     acc_index = 7; //Indice del json donde empiezan los datos
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

            string[] keys = new[]{ "Ax", "Ay", "Az", "Wx", "Wy", "Wz" };
            for( int i = 0; i < 6; i++ ){
                //"Ax":"5.2178231"
                string val = data[i + acc_index].Split( ':' )[1];

                //'5.2136789'
                //Quitar la comillas
                val = val.Substring( 2, val.Length - 3 );
                //Debug.Log(val);
                data[i + acc_index] = val;
            }

            //El indice 5 es el del tiempo
            string val2 = data[5].Split( ':' )[1];
            //'5.2136789'
            //Quitar la comillas
            val2      = val2.Substring( 2, val2.Length - 3 );
            last_time = float.Parse( val2 );

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
            Temp     = float.Parse( data[6] );
            Pressure = float.Parse( data[7] );
        }
    }

}