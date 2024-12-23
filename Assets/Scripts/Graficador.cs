using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using TMPro;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;

public class Graficador : MonoBehaviour{

    //Modificables por cualquiera
    public float      ValorMinimoY  = 0;
    public float      ValorMaximoY  = 100;
    public float      anchoTemporal = 10f;
    public Vector2Int Intervalos    = Vector2Int.one * 5;

    //Modificables por el programador
    public RectTransform FondoGrafica;

    public RectTransform ContenedorGrafica;
    public RectTransform ContentView;
    public Sprite        SpriteCriculo;

    public TextMeshProUGUI Titulo;
    public TextMeshProUGUI EtiquetaX;
    public TextMeshProUGUI EtiquetaY;

    public GameObject    TextoX;
    public GameObject    TextoY;
    public RectTransform GridX;
    public RectTransform GridY;

    public Vector2 tamañoGraficaInicial;

    private List< Vector2 > listaPuntos = new List< Vector2 >();

    private Vector2 separacionGrid;

    private float ultimaPosicionDeGridX;
    private int   cantidadGridX;

    private List< GameObject > objetos = new List< GameObject >();

    private void Start(){
        generarGrid();
    }

    public void setTituloYEtiquetas( string title, string labelX, string labelY ){
        Titulo.text    = title;
        EtiquetaX.text = labelX;
        EtiquetaY.text = labelY;
    }

    public void setYRangeAndXRange( float min, float max, float ancho = 10 ){
        ValorMinimoY  = min;
        ValorMaximoY  = max;
        anchoTemporal = ancho;
        generarGrid();
    }

    public void generarGrid(){
        tamañoGraficaInicial = FondoGrafica.sizeDelta;

        //Eliminar los textos anteriores
        foreach( Transform child in GridX ){
            if( child.gameObject == TextoX ){
                continue;
            }

            Destroy( child.gameObject );
        }

        foreach( Transform child in GridY ){
            if( child.gameObject == TextoY ){
                continue;
            }

            Destroy( child.gameObject );
        }

        TextoX.SetActive( true );
        TextoY.SetActive( true );
        separacionGrid = tamañoGraficaInicial / Intervalos;
        //Primero en X
        for( int i = 0; i <= Intervalos.x; i++ ){
            GameObject texto = Instantiate( TextoX, GridX );
            texto.GetComponent< RectTransform >().anchoredPosition = new Vector2(
                i * separacionGrid.x, texto.GetComponent< RectTransform >().anchoredPosition.y
            );

            texto.GetComponent< TextMeshProUGUI >().text = i == 0 ? "  0" : Math.Round( anchoTemporal * i / Intervalos.x, 2 ).ToString();
        }

        ultimaPosicionDeGridX = tamañoGraficaInicial.x;
        //Ahora en Y
        for( int i = 0; i <= Intervalos.y; i++ ){
            GameObject texto = Instantiate( TextoY, GridY );
            texto.GetComponent< RectTransform >().anchoredPosition = new Vector2(
                texto.GetComponent< RectTransform >().anchoredPosition.x, i * separacionGrid.y
            );

            texto.GetComponent< TextMeshProUGUI >().text =
                Math.Round( ValorMinimoY + ( ValorMaximoY - ValorMinimoY ) * i / Intervalos.y, 2 ).ToString();
        }

        cantidadGridX = GridX.childCount;

        TextoX.SetActive( false );
        TextoY.SetActive( false );
    }

    public void agregarPunto( float x, float y ){
        agregarPunto( new Vector2( x, y ) );
    }

    public void agregarPunto( Vector2 newPunto ){
        listaPuntos.Add( newPunto );
        //float y_max = ( from punto_ in listaPuntos select punto_.y ).Max();
        Vector2 punto = ( newPunto - new Vector2( 0, ValorMinimoY ) ) * tamañoGraficaInicial /
            new Vector2( anchoTemporal, ValorMaximoY - ValorMinimoY );

        objetos.Add( crearCirculo( punto ) );
        if( listaPuntos.Count > 1 ){
            objetos.Add(
                crearLinea(
                    ( listaPuntos[^2] - new Vector2( 0, ValorMinimoY ) ) * tamañoGraficaInicial /
                    new Vector2( anchoTemporal, ValorMaximoY - ValorMinimoY ),
                    punto
                )
            );
        }

        if( punto.x > ContentView.sizeDelta.x ){
            ContentView.sizeDelta = new Vector2( punto.x, ContentView.sizeDelta.y );
        }

        //Debug.Log( punto.x - ultimaPosicionDeGridX );
        //Debug.Log( separacionGrid.x );
        int contador = 0;
        while( punto.x - ultimaPosicionDeGridX >= 0 ){
            ultimaPosicionDeGridX += separacionGrid.x;
            GameObject texto = Instantiate( TextoX, GridX );
            texto.SetActive( true );
            texto.GetComponent< RectTransform >().anchoredPosition = new Vector2(
                ultimaPosicionDeGridX, texto.GetComponent< RectTransform >().anchoredPosition.y
            );
            //TODO: el -14 no deberia de existir idealmente
            texto.GetComponent< TextMeshProUGUI >().text =
                Math.Round( anchoTemporal * ( cantidadGridX++ - 1 ) / Intervalos.x - 12, 2 ).ToString();

            if( contador++ > 10 ){
                break;
            }

            objetos.Add( texto );
        }

        while( objetos.Count > 500 ){
            Destroy( objetos[0] );
            objetos.RemoveAt( 0 );
        }
    }

    public void graficar( List< Vector2 > puntos ){
        if( puntos.Count < 2 ){
            return;
        }

        foreach( Transform child in ContenedorGrafica ){
            Destroy( child.gameObject );
        }

        Vector2 delta = puntos[^1] - puntos[0];
        float   y_max = ( from punto in puntos select punto.y ).Max();

        for( int i = 0; i < puntos.Count; i++ ){
            Vector2 punto = ( puntos[i] - new Vector2( puntos[0].x, 0 ) ) * ( tamañoGraficaInicial / new Vector2( 1, y_max ) );

            crearCirculo( punto );
            if( i > 0 ){
                crearLinea( ( puntos[i - 1] - new Vector2( puntos[0].x, 0 ) ) * ( tamañoGraficaInicial / new Vector2( 1, y_max ) ), punto );
            }
        }
    }

    public GameObject crearCirculo( Vector2 position ){
        GameObject circulo = new GameObject( "punto", typeof(Image) );
        circulo.transform.SetParent( ContenedorGrafica, false );
        if( SpriteCriculo != null ){
            circulo.GetComponent< Image >().sprite = SpriteCriculo;
        }

        RectTransform rectTransform = circulo.GetComponent< RectTransform >();
        rectTransform.anchoredPosition = position;
        rectTransform.sizeDelta        = new Vector2( 5, 5 );
        rectTransform.anchorMax        = Vector2.zero;
        rectTransform.anchorMin        = Vector2.zero;
        return circulo;
    }

    public GameObject crearLinea( Vector2 inicio, Vector2 final ){
        Vector2 delta = final - inicio;
        if( delta == Vector2.zero ){
            delta = Vector2.down * 0.001f;
        }

        GameObject linea = new GameObject( "linea", typeof(Image) );
        linea.transform.SetParent( ContenedorGrafica, false );
        RectTransform rectTransform = linea.GetComponent< RectTransform >();
        rectTransform.anchoredPosition = ( inicio + final ) / 2;
        rectTransform.sizeDelta        = new Vector2( delta.magnitude, 2.5f );
        rectTransform.rotation =
            delta.x != 0 ? Quaternion.Euler( 0, 0, Mathf.Atan( delta.y / delta.x ) * Mathf.Rad2Deg ) :
                Quaternion.Euler( 0,            0, 90 );

        rectTransform.anchorMax = Vector2.zero;
        rectTransform.anchorMin = Vector2.zero;
        return linea;
    }
}