using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ManejadorGraficas : MonoBehaviour{
    public Graficador Grafica1;
    public Graficador Grafica2;
    public Graficador Grafica3;
    public Graficador Grafica4;
    public Graficador Grafica5;
    public Graficador Grafica6;
    public Graficador Grafica7;
    public Graficador Grafica8;
    public Graficador Grafica9;
    public Graficador Grafica10;

    private void Update(){
        float tiempo = Time.time * 1f;
        Grafica1.agregarPunto( new Vector2( tiempo,  1 + Random.Range( 0, 40 ) ) );
        Grafica6.agregarPunto( new Vector2( tiempo,  1 + Random.Range( 0, 40 ) ) );
        Grafica7.agregarPunto( new Vector2( tiempo,  1 + Random.Range( 0, 40 ) ) );
        Grafica8.agregarPunto( new Vector2( tiempo,  1 + Random.Range( 0, 40 ) ) );
        Grafica9.agregarPunto( new Vector2( tiempo,  1 + Random.Range( 0, 40 ) ) );
        Grafica10.agregarPunto( new Vector2( tiempo, 1 + Random.Range( 0, 40 ) ) );
        if( Comunicacion.NuevosDatos == false ){
            return;
        }

        Debug.Log( "graficando" );
        float timestamp = Comunicacion.DatosActuales.timestamp - Comunicacion.DatosIniciales.timestamp;
        Debug.Log( timestamp );
        Debug.Log( Comunicacion.DatosActuales.temperaturaSht );
        Grafica2.agregarPunto( new Vector2( timestamp, Comunicacion.DatosActuales.temperaturaSht ) );
        Grafica3.agregarPunto( new Vector2( timestamp, Comunicacion.DatosActuales.humedad ) );
        Grafica4.agregarPunto( new Vector2( timestamp, Comunicacion.DatosActuales.presion ) );
        Grafica5.agregarPunto( new Vector2( timestamp, Comunicacion.DatosActuales.altura ) );
        Comunicacion.NuevosDatos = false;
    }
}