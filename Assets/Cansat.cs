using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Threading;
using UnityEngine;

public class Cansat : MonoBehaviour{

    private static SerialPort _serialPort;
    Thread                    readThread = new Thread( Read );
    private bool              _isConnected;

    public static MPU6050 mpu6050 = new MPU6050();
    public static BMP280  bmp280  = new BMP280();

    private void Start(){
        _serialPort             = new SerialPort( "COM5", 57600 );
        _serialPort.ReadTimeout = 500;
        try{
            _serialPort.Open();
        }
        catch( Exception e ){
            return;
        }

        readThread.Start();
    }

    private void Update(){
        transform.Rotate( mpu6050.getDeltaAngle() );
        //transform.position += mpu6050.getDeltaPosition();
    }

    private void OnDisable(){
        readThread.Abort();
        _serialPort.Close();
    }

    public static void Read(){
        while( true ){
            try{
                string message = _serialPort.ReadLine();
                Debug.Log( message );
                mpu6050.update( message );
                //bmp280  = new BMP280( message );
            }
            catch( TimeoutException ){
            }
        }
    }

    public class MPU6050{
        public Vector3 Acc;  //m/s^2
        public Vector3 Gyro; //grad/s

        public void update( string message ){
            string[] data = message.Split( ',' );
            if( data.Length < 6 ){
                return;
            }

            Acc = new Vector3( float.Parse( data[0] ), float.Parse( data[1] ), float.Parse( data[2] ) );

            Gyro = new Vector3( float.Parse( data[3] ), float.Parse( data[4] ), float.Parse( data[5] ) );
        }

        public Vector3 getDeltaAngle( float deltaTime = 0.1f ){
            return new Vector3(
                Gyro.x / 40 * deltaTime,
                0,
                Gyro.y / 40 * deltaTime
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