using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ManejadorGraficas : MonoBehaviour{
    public Graficador Grafica1;
    public Graficador Grafica2;
    public Graficador Grafica3;
    public Graficador Grafica4;
    public Graficador Grafica5;

    private void Update(){
        float tiempo = Time.time * 1f;
        Grafica1.agregarPunto( new Vector2( tiempo, Random.Range( 0, 100 ) ) );
        if( Comunicacion.NuevosDatos == false ){
            return;
        }

        Debug.Log( "graficando" );
        Debug.Log( Comunicacion.DatosActuales.temperaturaSht );
        Grafica2.agregarPunto( new Vector2( Comunicacion.DatosActuales.timestamp, Comunicacion.DatosActuales.temperaturaSht ) );
        Grafica3.agregarPunto( new Vector2( Comunicacion.DatosActuales.timestamp, Comunicacion.DatosActuales.humedad ) );
        Grafica4.agregarPunto( new Vector2( Comunicacion.DatosActuales.timestamp, Comunicacion.DatosActuales.presion ) );
        Grafica5.agregarPunto( new Vector2( Comunicacion.DatosActuales.timestamp, Comunicacion.DatosActuales.altura ) );
        Comunicacion.NuevosDatos = false;
    }
}