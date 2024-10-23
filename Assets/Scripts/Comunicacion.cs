using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Threading;
using UnityEngine;

public class Comunicacion : MonoBehaviour{
    public string port = "COM6";
    public int baudrate = 9600;

    private SerialPort serialPort;
    public Datos DatosRecibidos = new Datos();

    public static bool NuevaImagen = true;
    public static bool NuevosDatos = false;
    public static Datos DatosActuales;
    public static Datos DatosIniciales = null;

    private Thread myThread;

    public List<Datos> DatosGuardados = new List<Datos>();

    private void Start(){
        //serialPort = new SerialPort( PlayerPrefs.GetString(ComSelector.key), 9600 );
        updatePort(port, baudrate);
    }

    private void updatePort(string portn, int baudraten){
        serialPort = new SerialPort(portn, baudraten);
        serialPort.ReadTimeout = 5000;
    }

    private void OnDisable(){
        if (myThread != null && myThread.IsAlive){
            myThread.Abort();
        }

        serialPort.Close();
    }

    private void Update(){
        if (serialPort.IsOpen){
            return;
        }

        try{
            Debug.Log("Tratando de abrir puerto serial");
            serialPort.Open();
            Debug.Log("Listo");
            // Iniciar el hilo
            myThread = new Thread(hiloLectura);
            myThread.Start();
        }
        catch (Exception e){
            Debug.Log(e);
        }
    }

    private void hiloLectura(){
        while (true){
            if (serialPort.IsOpen == false){
                return;
            }

            try{
                Refresh();
            }
            catch (Exception e){
                Debug.Log(e);
                serialPort.Close();
            }
        }
    }

    public void saveData(){
        string path = "Assets/Logs/log_" + DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss") + ".txt";
        Debug.Log("Guardando datos en " + path);
        using (StreamWriter sw = File.CreateText(path)){
            foreach (Datos d in DatosGuardados){
                sw.WriteLine(
                    $"{d.temperaturaSht}," +
                    $"{d.temperaturaMpu}," +
                    $"{d.timestamp}," +
                    $"{d.altura}," +
                    $"{d.presion}," +
                    $"{d.humedad}," +
                    $"{d.Aceleracion.x}," +
                    $"{d.Aceleracion.y}," +
                    $"{d.Aceleracion.z}," +
                    $"{d.Gyro.x}," +
                    $"{d.Gyro.y}," +
                    $"{d.Gyro.z}," +
                    $"{d.Magnet.x}," +
                    $"{d.Magnet.y}," +
                    $"{d.Magnet.z}," +
                    $"{d.direccionBrujula}," +
                    $"{d.yaw}," +
                    $"{d.pitch}," +
                    $"{d.roll}," +
                    $"{d.deltaFiltro}," +
                    $"{d.latitud}," +
                    $"{d.longitud}," +
                    $"{d.velocidadHorizontal}," +
                    $"{d.dilucionGps}," +
                    $"{d.tamañoImagen}" +
                    $"{d.bateria}"
                );
            }
        }
    }

    // Lee los datos seriales y realiza acciones en Unity
    private void Refresh(){
        if (DatosRecibidos.tamañoImagen == 0){
            string serialData = serialPort.ReadTo("\n");
            if (serialData.Length == 0){
                Debug.Log("No hay datos");
                return;
            }

            // Procesa los datos recibidos (puedes realizar análisis y acciones aquí)
            Debug.Log("Datos seriales recibidos: " + serialData);
            //Si el mensaje son los datos
            string[] data = serialData.Substring(1, serialData.Length - 2).Replace(" ", "").Split(',');
            if (data.Length < 25){
                Debug.Log("Datos incompletos");
                return;
            }

            DatosRecibidos = new Datos();
            float.TryParse(data[0], out DatosRecibidos.temperaturaSht);
            float.TryParse(data[1], out DatosRecibidos.temperaturaMpu);

            float.TryParse(data[2], out DatosRecibidos.timestamp);

            float.TryParse(data[3], out DatosRecibidos.altura);
            float.TryParse(data[4], out DatosRecibidos.presion);
            float.TryParse(data[5], out DatosRecibidos.humedad);

            float.TryParse(data[6], out DatosRecibidos.Aceleracion.x);
            float.TryParse(data[7], out DatosRecibidos.Aceleracion.y);
            float.TryParse(data[8], out DatosRecibidos.Aceleracion.z);

            float.TryParse(data[9], out DatosRecibidos.Gyro.x);
            float.TryParse(data[10], out DatosRecibidos.Gyro.y);
            float.TryParse(data[11], out DatosRecibidos.Gyro.z);

            float.TryParse(data[12], out DatosRecibidos.Magnet.x);
            float.TryParse(data[13], out DatosRecibidos.Magnet.y);
            float.TryParse(data[14], out DatosRecibidos.Magnet.z);

            float.TryParse(data[15], out DatosRecibidos.direccionBrujula);

            float.TryParse(data[16], out DatosRecibidos.yaw);
            float.TryParse(data[17], out DatosRecibidos.pitch);
            float.TryParse(data[18], out DatosRecibidos.roll);

            float.TryParse(data[19], out DatosRecibidos.deltaFiltro);

            float.TryParse(data[20], out DatosRecibidos.latitud);
            float.TryParse(data[21], out DatosRecibidos.longitud);

            float.TryParse(data[22], out DatosRecibidos.velocidadHorizontal);

            float.TryParse(data[23], out DatosRecibidos.dilucionGps);

            int.TryParse(data[24], out DatosRecibidos.tamañoImagen);

            int.TryParse(data[25], out DatosRecibidos.bateria);
            NuevosDatos = true;
            if (DatosIniciales == null){
                DatosIniciales = DatosRecibidos;
            }

            DatosGuardados.Add(DatosRecibidos);
        }
        else{
            /*
            byte[] buffer    = new byte[DatosRecibidos.tamañoImagen * 3];
            int    bytesRead = serialPort.Read( buffer, 0, buffer.Length );
            Debug.Log( $"imagen de {DatosRecibidos.tamañoImagen} bytes, {bytesRead}" );
            if( bytesRead > 0 ){
                File.WriteAllBytes( "Assets/Imagenes/imagen_serial.jpg",  buffer.Take( bytesRead ).ToArray() );
                File.WriteAllBytes( "Assets/Imagenes/imagen_serial2.jpg", buffer );
            }

            Debug.Log( buffer.ToString() );
            */

            try{
                byte[] imageData = new byte[DatosRecibidos.tamañoImagen];
                int x = 0;
                int maxIter = 1000;
                int iter;
                for (int i = 0; i < DatosRecibidos.tamañoImagen; i++){
                    iter = 0;

                    do{
                        x = serialPort.ReadByte();
                        iter++;
                        if (iter >= maxIter) Debug.LogError("Excedido el tiempo de espera para el serial !!");
                    } while (x == -1);

                    imageData[i] = (byte)x;
                }

                Debug.Log(String.Format("Buffer de {0} bytes leido", imageData.Length));
                Debug.Log(BitConverter.ToString(imageData));
                File.WriteAllBytes("Assets/Imagenes/imagen_serial.jpg", imageData);
                DatosRecibidos.tamañoImagen = 0;
                NuevaImagen = true;
            }
            catch (Exception e){
                Debug.Log(e.Message);
            }
        }

        DatosActuales = DatosRecibidos;
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

    public float latitud = 0;
    public float longitud = 0;

    public float velocidadHorizontal = 0;

    public float dilucionGps = 0;

    public int tamañoImagen = 0;

    public int bateria = 0;

    //Aqui acaba
    public float vibracion = 0;

    public float horizontal = 0;
    public float vertical = 0;

    public int numeroSatelites = 0;
    public int calidadSeñal = 0;
    public int señalRadio = 0;
}