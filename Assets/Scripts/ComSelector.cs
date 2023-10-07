using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class ComSelector : MonoBehaviour{
    public const string         key = "ComPort";
    public       TMP_InputField field;

    private void Start(){
        if( PlayerPrefs.HasKey( key ) == false ){
            PlayerPrefs.SetString( key, field.text );
        }

        field.text = PlayerPrefs.GetString( key );
        field.onEndEdit.AddListener( OnEndEdit );
    }

    private void OnEndEdit( string _arg0 ){
        PlayerPrefs.SetString( key, _arg0 );
    }
}