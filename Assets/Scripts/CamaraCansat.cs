using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO.Ports;

public class CamaraCansat : MonoBehaviour{

    public  Image     imagen;
    private Texture2D receivedImageTexture;

    private void Start(){
    }

    private void Update(){
        if( Comunicacion.NuevaImagen == false ){
            return;
        }

        Comunicacion.NuevaImagen = false;
        if( !LoadImageToUI() ){
            return;
        }

        Sprite sprite = Sprite.Create(
            receivedImageTexture, new Rect( 0, 0, receivedImageTexture.width, receivedImageTexture.height ), Vector2.one * 0.5f
        );

        imagen.sprite = sprite;
    }

    private bool LoadImageToUI(){
        byte[] imageData = System.IO.File.ReadAllBytes( "Assets/Imagenes/imagen_serial.jpg" );
        receivedImageTexture = new Texture2D( 2, 2 );
        receivedImageTexture.LoadRawTextureData( imageData );
        receivedImageTexture.Apply();
        //return receivedImageTexture.LoadImage( imageData );
        return true;
    }
}