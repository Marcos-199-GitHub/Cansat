using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO.Ports;

public class CamaraCansat : MonoBehaviour{

    public  Image     imagen;
    private Texture2D receivedImageTexture;

    private void Update(){
        if( Comunicacion.NuevaImagen == false ){
            return;
        }

        Comunicacion.NuevaImagen = false;
        if( !LoadImageToUI() ){
            Debug.Log( "no pude" );
            return;
        }

        Sprite sprite = Sprite.Create(
            receivedImageTexture, new Rect( 0, 0, receivedImageTexture.width, receivedImageTexture.height ), Vector2.one * 0.5f
        );

        imagen.sprite = sprite;
    }

    private bool LoadImageToUI(){
        System.Drawing.Image bitmap = System.Drawing.Bitmap.FromFile( "Assets/Imagenes/imagen_serial.jpg" );
        bitmap.Save( "Assets/Imagenes/imagen_serial.png", System.Drawing.Imaging.ImageFormat.Png );
        bitmap.Dispose();
        byte[] imageData = System.IO.File.ReadAllBytes( "Assets/Imagenes/imagen_serial.png" );
        receivedImageTexture = new Texture2D( 320, 240 );
        //receivedImageTexture.LoadImage( imageData );
        //receivedImageTexture.Apply();
        return receivedImageTexture.LoadImage( imageData );
    }
}