using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SceneDesigner : MonoBehaviour {

    public GameObject left_scale, left_Position_X, left_Position_Y, left_Position_Z, left_Rotation_X, left_Rotation_Y, left_Rotation_Z, left_Material, left_Mass;
    
    public GameObject right_scale, right_Position_X, right_Position_Y, right_Position_Z, right_Rotation_X, right_Rotation_Y, right_Rotation_Z, right_Material, right_Mass;

    float l_scale, l_mass, r_scale, r_mass;
    Vector3 l_pos, l_rot, r_pos, r_rot;
    string l_mat, r_mat;
    // Use this for initialization
    void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
		
	}

    public void AddModule()
    {
        CreateFrames();
    }
    public void RemoveModule()
    {

    }

    void CreateFrames()
    {
        //Left Frame:
        float.TryParse(left_scale.GetComponent<UnityEngine.UI.Text>().text, out l_scale);
        float.TryParse(left_Position_X.GetComponent<UnityEngine.UI.Text>().text, out l_pos.x);
        float.TryParse(left_Position_Y.GetComponent<UnityEngine.UI.Text>().text, out l_pos.y);
        float.TryParse(left_Position_Z.GetComponent<UnityEngine.UI.Text>().text, out l_pos.z);
        float.TryParse(left_Rotation_X.GetComponent<UnityEngine.UI.Text>().text, out l_rot.x);
        float.TryParse(left_Rotation_Y.GetComponent<UnityEngine.UI.Text>().text, out l_rot.y);
        float.TryParse(left_Rotation_Z.GetComponent<UnityEngine.UI.Text>().text, out l_rot.z);
        l_mat = left_Material.GetComponent<UnityEngine.UI.Text>().text;
        float.TryParse(left_Mass.GetComponent<UnityEngine.UI.Text>().text, out l_mass);
        
        //Right Frame:
        float.TryParse(right_scale.GetComponent<UnityEngine.UI.Text>().text, out r_scale);
        float.TryParse(right_Position_X.GetComponent<UnityEngine.UI.Text>().text, out r_pos.x);
        float.TryParse(right_Position_Y.GetComponent<UnityEngine.UI.Text>().text, out r_pos.y);
        float.TryParse(right_Position_Z.GetComponent<UnityEngine.UI.Text>().text, out r_pos.z);
        float.TryParse(right_Rotation_X.GetComponent<UnityEngine.UI.Text>().text, out r_rot.x);
        float.TryParse(right_Rotation_Y.GetComponent<UnityEngine.UI.Text>().text, out r_rot.y);
        float.TryParse(right_Rotation_Z.GetComponent<UnityEngine.UI.Text>().text, out r_rot.z);
        r_mat = right_Material.GetComponent<UnityEngine.UI.Text>().text;
        float.TryParse(right_Mass.GetComponent<UnityEngine.UI.Text>().text, out l_mass);




        Debug.Log(l_pos);
    }
}
