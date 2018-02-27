using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Camera_Movement : MonoBehaviour {

    public GameObject robot;
	// Use this for initialization
	void Start () {
        InvokeRepeating("getrobot", 0.5f, 99999);
        InvokeRepeating("getPos", 0.5f, 0.1f);
    }
	
	// Update is called once per frame
	void getPos () {
        this.transform.position = new Vector3(0, 15,robot.transform.position.z-10);
	}
    void getrobot()
    {
        robot = GameObject.Find("Frame");
    }


}
