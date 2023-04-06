using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Ensamble : MonoBehaviour{
    public Animator animator;

    public void play(){
        animator.SetTrigger( "play" );
    }
}