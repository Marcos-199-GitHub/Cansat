using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ManejadorGraficas : MonoBehaviour{
    public Graficador Grafica0;
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
    public Graficador Grafica11;
    public Graficador Grafica12;
    public Graficador Grafica13;
    public Graficador Grafica14;
    public Graficador Grafica15;
    public Graficador Grafica16;

    /*
        a) Temperatura interna y externa del CANSAT.
        b) Humedad relativa.
        c) Altitud.
        d) Longitud.
        e) Nivel de batería.
        f) Aceleración.
        g) Vibración.

     */
    private void Start(){
        Grafica0.setTituloYEtiquetas( "Random", "Tiempo [s]", "Algo" );
        Grafica1.setTituloYEtiquetas( "Temperatura interna en el tiempo", "Tiempo [s]", "Temperatura [°C]" );
        Grafica2.setTituloYEtiquetas( "Temperatura interna en la altura", "Altura [m]", "Temperatura [°C]" );
        Grafica3.setTituloYEtiquetas( "Temperatura externa en el tiempo", "Tiempo [s]", "Temperatura [°C]" );
        Grafica4.setTituloYEtiquetas( "Temperatura externa en la altura", "Altura [m]", "Temperatura [°C]" );

        Grafica5.setTituloYEtiquetas( "Humedad relativa en el tiempo", "Tiempo [s]", "Humedad relativa [%]" );
        Grafica6.setTituloYEtiquetas( "Humedad relativa en la altura", "Altura [m]", "Humedad relativa [%]" );

        Grafica7.setTituloYEtiquetas( "Altitud en el tiempo", "Tiempo [s]", "Altitud [m]" );
        Grafica8.setTituloYEtiquetas( "Altitud en la altura", "Altura [m]", "Altitud [m]" );

        Grafica9.setTituloYEtiquetas( "Longitud en el tiempo", "Tiempo [s]", "Longitud [°]" );
        Grafica10.setTituloYEtiquetas( "Longitud en la altura", "Altura [m]", "Longitud [°]" );

        Grafica11.setTituloYEtiquetas( "Nivel de batería en el tiempo", "Tiempo [s]", "Nivel de batería [%]" );
        Grafica12.setTituloYEtiquetas( "Nivel de batería en la altura", "Altura [m]", "Nivel de batería [%]" );

        Grafica13.setTituloYEtiquetas( "Aceleración en el tiempo", "Tiempo [s]", "Aceleración [m/s²]" );
        Grafica14.setTituloYEtiquetas( "Aceleración en la altura", "Altura [m]", "Aceleración [m/s²]" );

        Grafica15.setTituloYEtiquetas( "Vibración en el tiempo", "Tiempo [s]", "Vibración [m/s²]" );
        Grafica16.setTituloYEtiquetas( "Vibración en la altura", "Altura [m]", "Vibración [m/s²]" );

        Grafica0.setYRangeAndXRange( 0, 100 );

        Grafica1.setYRangeAndXRange( -20, 100 );
        Grafica2.setYRangeAndXRange( -20, 100 );
        Grafica3.setYRangeAndXRange( -20, 100 );
        Grafica4.setYRangeAndXRange( -20, 100 );

        Grafica5.setYRangeAndXRange( 0, 100 );
        Grafica6.setYRangeAndXRange( 0, 100 );
        
        Grafica7.setYRangeAndXRange( 2000, 2500 );
        Grafica8.setYRangeAndXRange( 2000, 2500 );
        
        Grafica9.setYRangeAndXRange( -100, 100 );
        Grafica10.setYRangeAndXRange( -100, 100 );
        
        Grafica11.setYRangeAndXRange( 0, 100 );
        Grafica12.setYRangeAndXRange( 0, 100 );
        
        Grafica13.setYRangeAndXRange( 0, 100 );
        Grafica14.setYRangeAndXRange( 0, 100 );
        
        Grafica15.setYRangeAndXRange( 0, 100 );
        Grafica16.setYRangeAndXRange( 0, 100 );
    }

    private void Update(){
        Grafica0.agregarPunto( Time.time, UnityEngine.Random.Range( 0, 100 ) );
        if( Comunicacion.NuevosDatos == false ){
            return;
        }

        float timestamp = Comunicacion.DatosActuales.timestamp - Comunicacion.DatosIniciales.timestamp;
        float altura    = Comunicacion.DatosActuales.altura    - Comunicacion.DatosIniciales.altura;

        Grafica1.agregarPunto( timestamp, Comunicacion.DatosActuales.temperaturaMpu );
        Grafica2.agregarPunto( altura, Comunicacion.DatosActuales.temperaturaMpu );
        Grafica3.agregarPunto( timestamp, Comunicacion.DatosActuales.temperaturaSht );
        Grafica4.agregarPunto( altura, Comunicacion.DatosActuales.temperaturaSht );
        Grafica5.agregarPunto( timestamp, Comunicacion.DatosActuales.humedad );
        Grafica6.agregarPunto( altura, Comunicacion.DatosActuales.humedad );
        Grafica7.agregarPunto( timestamp, Comunicacion.DatosActuales.altura );
        Grafica8.agregarPunto( altura, Comunicacion.DatosActuales.altura );
        Grafica9.agregarPunto( timestamp, Comunicacion.DatosActuales.longitud );
        Grafica10.agregarPunto( altura, Comunicacion.DatosActuales.longitud );
        Grafica11.agregarPunto( timestamp, Comunicacion.DatosActuales.bateria );
        Grafica12.agregarPunto( altura, Comunicacion.DatosActuales.bateria );
        Grafica13.agregarPunto( timestamp, Comunicacion.DatosActuales.Aceleracion.magnitude );
        Grafica14.agregarPunto( altura, Comunicacion.DatosActuales.Aceleracion.magnitude );
        Grafica15.agregarPunto( timestamp, Comunicacion.DatosActuales.vibracion );
        Grafica16.agregarPunto( altura, Comunicacion.DatosActuales.vibracion );

        Comunicacion.NuevosDatos = false;
    }
}