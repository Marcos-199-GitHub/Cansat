using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Modulos : MonoBehaviour{

    public  Ensamble ensamble;
    private int      x = 0;

    private void Start(){
        transform.position = new Vector3( x, 0.3f, 0 );
    }

    public void left(){
        if( x >= 0 ){
            x = 0;
            return;
        }

        x                  += 4;
        transform.position =  new Vector3( x, 0.3f, 0 );
    }

    public void right(){
        if( x <= -20 ){
            x = -20;
            ensamble.play();
            return;
        }

        x                  -= 4;
        transform.position =  new Vector3( x, 0.3f, 0 );
    }
}