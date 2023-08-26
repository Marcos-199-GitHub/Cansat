using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using Palmmedia.ReportGenerator.Core.Common;
using UnityEngine;
using UnityEngine.Serialization;

public class Comunicacion : MonoBehaviour{
    SerialPort   serialPort     = new SerialPort( "COM3", 9600 );
    public Datos DatosRecibidos = new Datos();

    private void Start(){
        try{
            serialPort.Open();
        }
        catch( Exception e ){
            Debug.LogError( e );
        }

        Debug.Log( JsonUtility.ToJson( DatosRecibidos ) );
    }

    // Lee los datos seriales y realiza acciones en Unity
    private void Update(){
        if( serialPort.IsOpen ){
            string serialData = serialPort.ReadTo( "\n" );
            // Procesa los datos recibidos (puedes realizar análisis y acciones aquí)
            Debug.Log( "Datos seriales recibidos: " + serialData );
            DatosRecibidos = JsonUtility.FromJson< Datos >( serialData );
        }
        else{
        }
    }
}

[Serializable]
public class Datos{
    public float humedad     = 1.1f;
    public float temperatura = 1.2f;
    public float presion     = 1.3f;
    public float vibracion   = 1.4f;
    public float latitud     = 1.5f;
    public float longitud    = 1.6f;
    public float altura      = 1.7f;
    public float horizontal  = 1.8f;
    public float vertical    = 1.9f;
    public float roll        = 2.1f;
    public float pitch       = 2.2f;
    public float yaw         = 2.3f;

    public int numeroSatelites = 11;
    public int calidadSeñal    = 3;
    public int señalRadio      = 2;
    public int bateria         = 50;
}