using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Threading;
using TMPro;
using UnityEngine;

public class Altimetro : MonoBehaviour{
    public string port = "COM6";
    public int baudrate = 9600;

    private SerialPort serialPort;
    public DatosAltimetro DatosRecibidos = new DatosAltimetro();
    public List<DatosAltimetro> DatosGuardados = new List<DatosAltimetro>();

    public TextMeshProUGUI TextTemperatura;
    public TextMeshProUGUI TextPresion;
    public TextMeshProUGUI TextAltura;
    public TextMeshProUGUI TextVelocidad;

    public Graficador GraficaTemperatura;
    public Graficador GraficaPresion;
    public Graficador GraficaAltura;
    public Graficador GraficaVelocidad;

    public float t0 = 0;

    private void Start(){
        //serialPort = new SerialPort( PlayerPrefs.GetString(ComSelector.key), 9600 );
        updatePort(port, baudrate);

        GraficaTemperatura.setTituloYEtiquetas("Temperatura BMP180", "Tiempo [s]", "Temperatura [°C]");
        GraficaPresion.setTituloYEtiquetas("Presión BMP180", "Tiempo [s]", "Presión [hPa]");
        GraficaAltura.setTituloYEtiquetas("Altura BMP180", "Tiempo [s]", "Altura [m]");
        GraficaVelocidad.setTituloYEtiquetas("Velocidad BMP180", "Tiempo [s]", "Velocidad [m/s]");

        GraficaTemperatura.setYRangeAndXRange(0, 40);
        GraficaPresion.setYRangeAndXRange(0, 1200);
        GraficaAltura.setYRangeAndXRange(2000, 3000);
        GraficaVelocidad.setYRangeAndXRange(-10, 10);

        t0 = Time.time;
    }

    private void updatePort(string portn, int baudraten){
        serialPort = new SerialPort(portn, baudraten);
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

            // Procesa los datos recibidos (puedes realizar análisis y acciones aquí)
            Debug.Log("Datos seriales recibidos: " + serialData);
            //Si el mensaje son los datos
            string[] data = serialData.Replace(" ", "").Split(',');
            if (data.Length < 4){
                Debug.Log("Datos incompletos");
                return;
            }

            DatosRecibidos = new DatosAltimetro();
            DatosRecibidos.tiempo = Time.time - t0;
            float.TryParse(data[0], out DatosRecibidos.temperatura);
            float.TryParse(data[1], out DatosRecibidos.presion);
            float.TryParse(data[2], out DatosRecibidos.altura);
            float.TryParse(data[3], out DatosRecibidos.velocidad);
            DatosGuardados.Add(DatosRecibidos);

            TextTemperatura.text = DatosRecibidos.temperatura + " °C";
            TextPresion.text = DatosRecibidos.presion + " hPa";
            TextAltura.text = DatosRecibidos.altura + " m";
            TextVelocidad.text = DatosRecibidos.velocidad + " m/s";

            GraficaTemperatura.agregarPunto(DatosRecibidos.tiempo, DatosRecibidos.temperatura);
            GraficaPresion.agregarPunto(DatosRecibidos.tiempo, DatosRecibidos.presion);
            GraficaAltura.agregarPunto(DatosRecibidos.tiempo, DatosRecibidos.altura);
            GraficaVelocidad.agregarPunto(DatosRecibidos.tiempo, DatosRecibidos.velocidad);

            return;
        }

        try{
            Debug.Log("Tratando de abrir puerto serial");
            serialPort.Open();
            Debug.Log("Listo");
            t0 = Time.time;
        }
        catch (Exception e){
            Debug.Log(e);
        }
    }

    public void saveData(){
        string path = "Assets/Logs/log_" + DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss") + ".txt";
        Debug.Log("Guardando datos en " + path);
        using (StreamWriter sw = File.CreateText(path)){
            foreach (DatosAltimetro d in DatosGuardados){
                sw.WriteLine(d.tiempo + "," + d.temperatura + "," + d.presion + "," + d.altura + "," + d.velocidad);
            }
        }
    }
}

[Serializable]
public class DatosAltimetro{
    public float tiempo;
    public float temperatura;
    public float presion;
    public float altura;
    public float velocidad;
}