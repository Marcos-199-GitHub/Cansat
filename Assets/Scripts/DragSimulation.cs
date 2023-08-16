using System.Collections;
using System.Collections.Generic;
using System.Security.Cryptography;
using UnityEngine;

//mg-(1/2)(Cd)pAV^2




public class DragSimulation : MonoBehaviour
{

    public float airDensity = 1.225f;
    public float area = 0.0f;
    public float dragCoefficient = 0.0f;
    public int isOpen = 0;

    Rigidbody rb;
    Cansat.DataRecived telemetria;
    // Start is called before the first frame update
    void Start()
    {
         rb = this.GetComponent<Rigidbody>();
        telemetria = Cansat.dataRecived; 
    }

    // Update is called once per frame
    void Update()
    {
        float _airResistance =  isOpen * dragCoefficient * ((airDensity * area) / 2) * (rb.velocity.sqrMagnitude);
        Vector2 _dragForce = _airResistance * -rb.velocity.normalized;
        rb.AddForce(_dragForce, ForceMode.Force);
        //Debug.Log(telemetria.rawMessage);
    }
}
