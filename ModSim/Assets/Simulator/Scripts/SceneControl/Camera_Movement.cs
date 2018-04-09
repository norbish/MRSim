/* Unity, movement of camera (Camera_Movement)
 * Torstein Sundnes Lenerand
 * NTNU Ålesund
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Camera_Movement : MonoBehaviour {

    public GameObject robot;
    public Button r_left, r_right;
	// Use this for initialization
	void Start () {
        
    }
    public void Initialize()
    {
        CancelInvoke("getrobot");CancelInvoke("getpos");
        InvokeRepeating("getrobot", 0.1f, 99999);
        InvokeRepeating("getPos", 0.5f, 0.01f);
    }

    

    // Update is called once per frame
    float rot = 0;
	void getPos () {

        //this.transform.position = new Vector3(robot.transform.position.x, robot.transform.position.y + 6,robot.transform.position.z-10);
        if (robot != null)
        {
            //this.transform.position = new Vector3(robot.transform.position.x, robot.transform.position.y+6, robot.transform.position.z-5);
            this.transform.RotateAround(robot.transform.position, Vector3.up, 0);
            this.transform.LookAt(robot.transform.position);
        }
        else
        {
            getrobot();
        }
    }
    /*private void Update()
    {
        this.transform.eulerAngles = new Vector3(30,0,0);
        this.transform.position = new Vector3(this.transform.position.x, 14, this.transform.position.z);
    }*/
    void getrobot()
    {
        robot = GameObject.Find("Frame");
        if (robot == null)
            robot = GameObject.Find("SensorModule");
        if (robot == null)
        { robot = new GameObject(); robot.transform.position = new Vector3(0, 18, -10); }

        this.transform.position = new Vector3(robot.transform.position.x, robot.transform.position.y + 6, robot.transform.position.z - 10);
        /*this.transform.position = robot.transform.position;
        this.transform.position += new Vector3(0,6,-10);
        this.transform.SetParent(robot.transform);
        this.transform.eulerAngles = new Vector3(30, 0, 0);*/
    }

    public Scrollbar scroll;
    public void Zoom()
    {
        this.GetComponent<Camera>().fieldOfView = 80 * scroll.value + 1;
    }

    public void R_left()
    {
        this.transform.RotateAround(robot.transform.position, Vector3.up, 30);

    }
    public void R_right()
    {
        this.transform.RotateAround(robot.transform.position, Vector3.up, -30);

    }


}
