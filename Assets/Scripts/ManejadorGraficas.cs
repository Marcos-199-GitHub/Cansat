using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

public class ManejadorGraficas : MonoBehaviour{
    public Graficador Grafica0;

    public Graficador GraficaTempEnT;
    public Graficador GraficaTempEnA;

    public Graficador GraficaTempinEnT;
    public Graficador GraficaTempinEnA;

    public Graficador GraficaHumEnT;
    public Graficador GraficaHumEnA;

    public Graficador GraficaAEnT;
    public Graficador Grafica8;

    public Graficador GraficaLonEnT;
    public Graficador GraficaLonEnA;

    public Graficador GraficaLatEnT;
    public Graficador GraficaLatEnA;

    public Graficador GraficaBatEnT;
    public Graficador GraficaBatEnA;

    public Graficador GraficaAccEnT;
    public Graficador GraficaAccEnA;

    public Graficador GraficaVibEnT;
    public Graficador GraficaVibEnA;

    /*
        a) Temperatura interna y externa del CANSAT.
        b) Humedad relativa.
        c) Altitud.
        d) Longitud.
        e) Nivel de batería.
        f) Aceleración.
        g) Vibración.

     */
    private int i = 0;

    private void Start(){
        Grafica0.setTituloYEtiquetas( "Random", "Tiempo [s]", "Algo" );
        GraficaTempEnT.setTituloYEtiquetas( "Temperatura interna en el tiempo", "Tiempo [s]", "Temperatura [°C]" );
        GraficaTempEnA.setTituloYEtiquetas( "Temperatura interna en la altura", "Altura [m]", "Temperatura [°C]" );

        GraficaTempinEnT.setTituloYEtiquetas( "Temperatura externa en el tiempo", "Tiempo [s]", "Temperatura [°C]" );
        GraficaTempinEnA.setTituloYEtiquetas( "Temperatura externa en la altura", "Altura [m]", "Temperatura [°C]" );

        GraficaHumEnT.setTituloYEtiquetas( "Humedad relativa en el tiempo", "Tiempo [s]", "Humedad relativa [%]" );
        GraficaHumEnA.setTituloYEtiquetas( "Humedad relativa en la altura", "Altura [m]", "Humedad relativa [%]" );

        GraficaAEnT.setTituloYEtiquetas( "Altitud en el tiempo", "Tiempo [s]", "Altitud [m]" );
        //Grafica8.setTituloYEtiquetas( "Altitud en la altura", "Altura [m]", "Altitud [m]" );

        GraficaLonEnT.setTituloYEtiquetas( "Longitud en el tiempo", "Tiempo [s]", "Longitud [°]" );
        GraficaLonEnA.setTituloYEtiquetas( "Longitud en la altura", "Altura [m]", "Longitud [°]" );

        GraficaLatEnT.setTituloYEtiquetas( "Latitud en el tiempo", "Tiempo [s]", "Latitud [°]" );
        GraficaLatEnA.setTituloYEtiquetas( "Latitud en la altura", "Altura [m]", "Latitud [°]" );

        GraficaBatEnT.setTituloYEtiquetas( "Nivel de batería en el tiempo", "Tiempo [s]", "Nivel de batería [%]" );
        GraficaBatEnA.setTituloYEtiquetas( "Nivel de batería en la altura", "Altura [m]", "Nivel de batería [%]" );

        GraficaAccEnT.setTituloYEtiquetas( "Aceleración en el tiempo", "Tiempo [s]", "Aceleración [m/s²]" );
        GraficaAccEnA.setTituloYEtiquetas( "Aceleración en la altura", "Altura [m]", "Aceleración [m/s²]" );

        GraficaVibEnT.setTituloYEtiquetas( "Vibración en el tiempo", "Tiempo [s]", "Vibración [m/s²]" );
        GraficaVibEnA.setTituloYEtiquetas( "Vibración en la altura", "Altura [m]", "Vibración [m/s²]" );

        Grafica0.setYRangeAndXRange( 0, 100 );

        GraficaTempEnT.setYRangeAndXRange( -20, 100 );
        GraficaTempEnA.setYRangeAndXRange( -20, 100 );

        GraficaTempinEnT.setYRangeAndXRange( -20, 100 );
        GraficaTempinEnA.setYRangeAndXRange( -20, 100 );

        GraficaHumEnT.setYRangeAndXRange( 0, 100 );
        GraficaHumEnA.setYRangeAndXRange( 0, 100 );

        GraficaAEnT.setYRangeAndXRange( 2000, 2500 );
        //Grafica8.setYRangeAndXRange( 2000, 2500 );

        GraficaLonEnT.setYRangeAndXRange( -100, 100 );
        GraficaLonEnA.setYRangeAndXRange( -100, 100 );

        GraficaLatEnT.setYRangeAndXRange( -100, 100 );
        GraficaLatEnA.setYRangeAndXRange( -100, 100 );

        GraficaBatEnT.setYRangeAndXRange( 0, 100 );
        GraficaBatEnA.setYRangeAndXRange( 0, 100 );

        GraficaAccEnT.setYRangeAndXRange( 0, 100 );
        GraficaAccEnA.setYRangeAndXRange( 0, 100 );

        GraficaVibEnT.setYRangeAndXRange( 0, 100 );
        GraficaVibEnA.setYRangeAndXRange( 0, 100 );

        Grafica8.gameObject.SetActive( false );
    }

    private void Update(){
        if( i >= 3 ){
            Grafica0.agregarPunto( Time.time, UnityEngine.Random.Range( 0, 100 ) );
            i = 0;
        }

        i++;
        if( Comunicacion.NuevosDatos == false ){
            return;
        }

        float timestamp = Comunicacion.DatosActuales.timestamp - Comunicacion.DatosIniciales.timestamp;
        float altura    = Comunicacion.DatosActuales.altura    - Comunicacion.DatosIniciales.altura;

        GraficaTempEnT.agregarPunto( timestamp, Comunicacion.DatosActuales.temperaturaMpu );
        GraficaTempEnA.agregarPunto( altura, Comunicacion.DatosActuales.temperaturaMpu );

        GraficaTempinEnT.agregarPunto( timestamp, Comunicacion.DatosActuales.temperaturaSht );
        GraficaTempinEnA.agregarPunto( altura, Comunicacion.DatosActuales.temperaturaSht );

        GraficaHumEnT.agregarPunto( timestamp, Comunicacion.DatosActuales.humedad );
        GraficaHumEnA.agregarPunto( altura, Comunicacion.DatosActuales.humedad );

        GraficaAEnT.agregarPunto( timestamp, Comunicacion.DatosActuales.altura );
        //Grafica8.agregarPunto( altura, Comunicacion.DatosActuales.altura );

        GraficaLonEnT.agregarPunto( timestamp, Comunicacion.DatosActuales.longitud );
        GraficaLonEnA.agregarPunto( altura, Comunicacion.DatosActuales.longitud );

        GraficaLatEnT.agregarPunto( timestamp, Comunicacion.DatosActuales.latitud );
        GraficaLatEnA.agregarPunto( altura, Comunicacion.DatosActuales.latitud );

        GraficaBatEnT.agregarPunto( timestamp, Comunicacion.DatosActuales.bateria );
        GraficaBatEnA.agregarPunto( altura, Comunicacion.DatosActuales.bateria );

        GraficaAccEnT.agregarPunto( timestamp, Comunicacion.DatosActuales.Aceleracion.magnitude );
        GraficaAccEnA.agregarPunto( altura, Comunicacion.DatosActuales.Aceleracion.magnitude );

        GraficaVibEnT.agregarPunto( timestamp, Comunicacion.DatosActuales.vibracion );
        GraficaVibEnA.agregarPunto( altura, Comunicacion.DatosActuales.vibracion );

        Comunicacion.NuevosDatos = false;
    }

    private void guardarGraficas(){
    }
}