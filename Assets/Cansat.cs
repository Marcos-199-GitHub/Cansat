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

    public static MPU6050 mpu6050 = new MPU6050();
    public static BMP280  bmp280  = new BMP280();

    private void Start(){
        _serialPort             = new SerialPort( ".\\COM4", 9600 );
        _serialPort.ReadTimeout = 500;
        try{
            Debug.Log( "Abriendo puerto serial" );
            _serialPort.Open();
            Debug.Log( "Listo" );
            //Debug.Log(s);
        }
        catch( Exception e ){
            Debug.Log( "Error" );
            Debug.Log( e );
            return;
        }

        readThread.Start();
    }

    private void Update(){
        if( !_serialPort.IsOpen ){
            Debug.Log( "Serial is closed" );
        } //comprobamos que el puerto esta abierto

        transform.Rotate( mpu6050.getDeltaAngle() );
        //transform.position += mpu6050.getDeltaPosition();
    }

    private void OnDisable(){
        readThread.Abort();
        _serialPort.Close();
    }

    public static void Read(){
        Debug.Log( "Thread de lectura iniciado" );
        while( true ){
            //Debug.Log("Leyendo");
            try{
                string message = _serialPort.ReadTo( "\n" );
                mpu6050.update( message );
                //bmp280  = new BMP280( message );
            }
            catch( TimeoutException ){
            }
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
            string[] data = message.Split( ',' );
            //Debug.Log(data);
            if( data.Length < 6 ){
                return;
            }

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