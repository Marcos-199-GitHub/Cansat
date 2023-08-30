using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO.Ports;

public class CamaraCansat : MonoBehaviour{

    public bool por_serial; //?

    public string filePath = "Assets/Imagenes/imagen_serial.bin"; // Ruta relativa al archivo binario
    public Image  imagen;

    private SerialPort serialPort;
    private Texture2D  receivedImageTexture;

    private void Start(){
        if( !por_serial ){
            return;
        }

        serialPort = new SerialPort( "COM3", 9600 );
        serialPort.Open();
    }

    private void OnDestroy(){
        if( !por_serial ){
            return;
        }

        if( serialPort != null && serialPort.IsOpen ){
            serialPort.Close();
        }
    }

    private void Update(){
        if( por_serial ? !RecieveImageToUI() : !LoadImageToUI() ){
            return;
        }

        Sprite sprite = Sprite.Create(
            receivedImageTexture, new Rect( 0, 0, receivedImageTexture.width, receivedImageTexture.height ), Vector2.one * 0.5f
        );

        imagen.sprite = sprite;
    }

    private bool RecieveImageToUI(){
        if( serialPort.IsOpen && serialPort.BytesToRead > 0 ){
            byte[] imageData = new byte[640 * 480 * 3]; //ancho y alto de imagen
            serialPort.Read( imageData, 0, imageData.Length );
            receivedImageTexture.LoadRawTextureData( imageData );
            receivedImageTexture.Apply();
            return true;
        }

        return false;
    }

    private bool LoadImageToUI(){
        byte[] imageData = System.IO.File.ReadAllBytes( "Assets/Imagenes/imagen_serial.jpg" );
        receivedImageTexture = new Texture2D( 2, 2 );
        return receivedImageTexture.LoadImage( imageData );
    }
}