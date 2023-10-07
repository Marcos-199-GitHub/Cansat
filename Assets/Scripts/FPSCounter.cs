using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class FPSCounter : MonoBehaviour{
    private float           fps       = 0.0f;
    private int             muestreos = 10;
    private int             i         = 0;
    public  TextMeshProUGUI texto;

    void Update(){
        fps += 1 / Time.deltaTime;
        if( i == muestreos ){
            fps        /= muestreos;
            texto.text =  "FPS: "+ Mathf.RoundToInt( fps );
            fps        =  0;
            i          =  0;
        }

        i++;
    }
}