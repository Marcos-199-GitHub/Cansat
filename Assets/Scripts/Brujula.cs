using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using TMPro;
using UnityEngine;

public class Brujula : MonoBehaviour{
    public string port = "COM6";

    public int baudrate = 9600;

    private SerialPort serialPort;

    public TextMeshProUGUI TextAngulo;
    public RectTransform flecha;

    private void Start(){
        serialPort = new SerialPort(port, baudrate);
        serialPort.ReadTimeout = 5000;
    }

    private void OnDisable(){
        serialPort.Close();
    }

    private void Update(){
        if (serialPort.IsOpen){
            string serialData = serialPort.ReadTo("\n");
            if (serialData.Length == 0){
                Debug.Log("No hay datos");
                return;
            }

            float angulo = float.Parse(serialData);
            flecha.rotation = Quaternion.Euler(0, 0, angulo);
            TextAngulo.text = angulo.ToString("F2") + "°";
        }
        else{
            try{
                serialPort.Open();
            }
            catch (Exception e){
                Debug.Log("Error al abrir el puerto: " + e.Message);
            }
        }
    }
}