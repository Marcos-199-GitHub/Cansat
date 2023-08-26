using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class MostrarDatos : MonoBehaviour{
    public Comunicacion comunicacion;

    public TextMeshProUGUI humedad;
    public TextMeshProUGUI temperatura;
    public TextMeshProUGUI presion;
    public TextMeshProUGUI vibracion;
    public TextMeshProUGUI latitud;
    public TextMeshProUGUI longitud;
    public TextMeshProUGUI altitud;
    public TextMeshProUGUI horizontal;
    public TextMeshProUGUI vertical;
    public TextMeshProUGUI roll;
    public TextMeshProUGUI pitch;
    public TextMeshProUGUI yaw;
    public Image           yawImage;
    public TextMeshProUGUI numeroSatelites;
    public Image           calidadSeñalImage;
    public Image           señalRadioImage;
    public TextMeshProUGUI bateria;

    private void Update(){
        humedad.text                              = comunicacion.DatosRecibidos.humedad + "%";
        temperatura.text                          = comunicacion.DatosRecibidos.temperatura + " °C";
        presion.text                              = comunicacion.DatosRecibidos.presion + " hPa";
        vibracion.text                            = comunicacion.DatosRecibidos.vibracion + " dB";
        latitud.text                              = "Latitud: " + comunicacion.DatosRecibidos.latitud + "°";
        longitud.text                             = "Longitud: " + comunicacion.DatosRecibidos.longitud + "°";
        altitud.text                              = "Altura: " + comunicacion.DatosRecibidos.altura + " m";
        horizontal.text                           = "Horizontal: " + comunicacion.DatosRecibidos.horizontal + " m/s";
        vertical.text                             = "Vertical: " + comunicacion.DatosRecibidos.vertical + " m/s";
        roll.text                                 = "Roll (eje x): " + comunicacion.DatosRecibidos.roll + " °";
        pitch.text                                = "Pitch (eje y): " + comunicacion.DatosRecibidos.pitch + " °";
        yaw.text                                  = comunicacion.DatosRecibidos.yaw + " °";
        yawImage.transform.rotation               = Quaternion.Euler( 0, 0, comunicacion.DatosRecibidos.yaw );
        numeroSatelites.text                      = comunicacion.DatosRecibidos.numeroSatelites.ToString();
        calidadSeñalImage.rectTransform.sizeDelta = new Vector2( ( 4 - comunicacion.DatosRecibidos.calidadSeñal ) * 10, 0 );
        señalRadioImage.rectTransform.sizeDelta   = new Vector2( ( 4 - comunicacion.DatosRecibidos.señalRadio )   * 10, 0 );
        bateria.text                              = comunicacion.DatosRecibidos.bateria + "%";
    }
}