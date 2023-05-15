using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class IrAEscena : MonoBehaviour{
    public void irAMenu(){
        SceneManager.LoadScene( "MenuInicial" );
    }

    public void irAModulos(){
        SceneManager.LoadScene( "Modulos" );
    }

    public void irAEstacion(){
        SceneManager.LoadScene( "Estacion" );
    }
}