using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text.RegularExpressions;
using Palmmedia.ReportGenerator.Core.Common;
using UnityEngine;
using UnityEngine.Serialization;

public class Comunicacion : MonoBehaviour{
    SerialPort   serialPort;
    public Datos DatosRecibidos = new Datos();

    private void Start(){
        //serialPort = new SerialPort( PlayerPrefs.GetString(ComSelector.key), 9600 );
        serialPort             = new SerialPort( "COM6", 9600 );
        serialPort.ReadTimeout = 5000;
        try{
            Debug.Log( "Tratando de abrir puerto serial" );
            serialPort.Open();
            Debug.Log( "Listo" );
        }
        catch( Exception e ){
            Debug.LogError( e );
        }
    }

    private void OnDisable(){
        serialPort.Close();
    }

    // Lee los datos seriales y realiza acciones en Unity
    private void Update(){
        if( !serialPort.IsOpen ){
            return;
        }

        if( DatosRecibidos.tamañoImagen == 0 ){
            string serialData = serialPort.ReadTo( "\n" );
            // Procesa los datos recibidos (puedes realizar análisis y acciones aquí)
            Debug.Log( "Datos seriales recibidos: " + serialData );
            //Si el mensaje son los datos
            string[] data = serialData.Substring( 1, serialData.Length - 2 ).Replace( " ", "" ).Split( ',' );
            DatosRecibidos = new Datos();
            float.TryParse( data[0], out DatosRecibidos.temperaturaSht );
            float.TryParse( data[1], out DatosRecibidos.temperaturaMpu );

            float.TryParse( data[2], out DatosRecibidos.presion );
            float.TryParse( data[3], out DatosRecibidos.altura );
            float.TryParse( data[4], out DatosRecibidos.timestamp );
            float.TryParse( data[5], out DatosRecibidos.humedad );

            float.TryParse( data[6], out DatosRecibidos.Aceleracion.x );
            float.TryParse( data[7], out DatosRecibidos.Aceleracion.y );
            float.TryParse( data[8], out DatosRecibidos.Aceleracion.z );

            float.TryParse( data[9],  out DatosRecibidos.Gyro.x );
            float.TryParse( data[10], out DatosRecibidos.Gyro.y );
            float.TryParse( data[11], out DatosRecibidos.Gyro.z );

            float.TryParse( data[12], out DatosRecibidos.Magnet.x );
            float.TryParse( data[13], out DatosRecibidos.Magnet.y );
            float.TryParse( data[14], out DatosRecibidos.Magnet.z );

            float.TryParse( data[15], out DatosRecibidos.direccionBrujula );

            float.TryParse( data[16], out DatosRecibidos.yaw );
            float.TryParse( data[17], out DatosRecibidos.pitch );
            float.TryParse( data[18], out DatosRecibidos.roll );

            float.TryParse( data[19], out DatosRecibidos.deltaFiltro );

            float.TryParse( data[20], out DatosRecibidos.latitud );
            float.TryParse( data[21], out DatosRecibidos.longitud );

            float.TryParse( data[22], out DatosRecibidos.velocidadHorizontal );

            float.TryParse( data[23], out DatosRecibidos.dilucionGps );

            int.TryParse( data[24], out DatosRecibidos.tamañoImagen );
        }
        else{
            byte[] buffer    = new byte[DatosRecibidos.tamañoImagen];
            int    bytesRead = serialPort.Read( buffer, 0, buffer.Length );
            Debug.Log( $"imagen de {DatosRecibidos.tamañoImagen} bytes, {bytesRead}" );
            if( bytesRead > 0 ){
                File.WriteAllBytes( "Assets/Imagenes/imagen_serial.jpg", buffer.Take( bytesRead ).ToArray() );
                //File.WriteAllBytes("Assets/Imagenes/imagen_serial.jpg", buffer);
            }

            Debug.Log( buffer.ToString() );
            DatosRecibidos.tamañoImagen = 0;
        }
    }
}

[Serializable]
public class Datos{
    public float temperaturaSht;
    public float temperaturaMpu;
    public float presion;
    public float altura;
    public float timestamp;
    public float humedad;

    public Vector3 Aceleracion;
    public Vector3 Gyro;
    public Vector3 Magnet;

    public float direccionBrujula;

    public float yaw;
    public float pitch;
    public float roll;

    public float deltaFiltro = 0;

    public float latitud  = 0;
    public float longitud = 0;

    public float velocidadHorizontal = 0;

    public float dilucionGps = 0;

    public int tamañoImagen = 0;

    //Aqui acaba
    public float vibracion = 0;

    public float horizontal = 0;
    public float vertical   = 0;

    public int numeroSatelites = 0;
    public int calidadSeñal    = 0;
    public int señalRadio      = 0;
    public int bateria         = 0;

}