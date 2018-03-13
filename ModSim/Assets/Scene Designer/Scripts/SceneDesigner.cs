using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Simulation_Core;
using System;
using System.Xml.Serialization;
using System.IO;

public class SceneDesigner : MonoBehaviour {
    public GameObject SIMULATOR,CAMERA;

    public GameObject left_scale, left_Position_X, left_Position_Y, left_Position_Z, left_Rotation_X, left_Rotation_Y, left_Rotation_Z, left_Material, left_Mass;
    
    public GameObject right_scale, right_Position_X, right_Position_Y, right_Position_Z, right_Rotation_X, right_Rotation_Y, right_Rotation_Z, right_Material, right_Mass;

    public GameObject leftFrame_name, rightFrame_name;


    float l_scale, l_mass, r_scale, r_mass;
    Vector3 l_pos, l_rot, r_pos, r_rot;
    string l_mat, r_mat;


    string dir = "";
    string upperFrame_ObjName = "upper.obj";
    string bottomFrame_ObjName = "bottom.obj";

    Robot robot_serialize = new Robot();

    // Use this for initialization
    void Start ()
    {
        dir = Application.streamingAssetsPath + "/Robot/";//Get the path of the streaming assets
        upperFrame_ObjName = leftFrame_name.GetComponent<UnityEngine.UI.Text>().text;
        bottomFrame_ObjName = rightFrame_name.GetComponent<UnityEngine.UI.Text>().text;
    }
	
	// Update is called once per frame
	void Update () {
		
	}
    /*----------------------------------------------Initializing Simulation-----------------------------------------------*/
    /**-----------------------------------------------Starting simulation-------------------------------------------------*/
    public void StartSimulation()
    {
        Serialization();
        SIMULATOR.SendMessage("Main_Initialization");
        CAMERA.SendMessage("Initialize");

    }
    /**------------------------------------------------Serializing robot-------------------------------------------------*/
    void Serialization()//Make into static class?
    {
        robot_serialize = new Robot();

        List<Frame> frames = new List<Frame>();
        List<Simulation_Core.Joint> joints = new List<Simulation_Core.Joint>();

        //Start position for the first module
        Vector3 start = new Vector3(15, 12, 40);

        //For finding the size of the modules:
        ObjImporter import = new ObjImporter();

        Mesh leftMesh = import.ImportFile(dir + upperFrame_ObjName); Bounds leftBound = leftMesh.bounds;
        Mesh rightMesh = import.ImportFile(dir + bottomFrame_ObjName); Bounds rightBound = rightMesh.bounds;

        //Creating modules
        for (int i = 0; i < 10; i++)
        {
            //This, user should decide him/herself:
            var rot = i % 2 == 0 ? new Vector3(0, -Mathf.PI / 2, 0) : new Vector3(0, -Mathf.PI / 2, -Mathf.PI / 2);

            Frame f1 = DefineFrame("Box", start, 10, rot, 50, "Plastic");
            Frame f2 = DefineFrame("Box", start, 10, rot, 50, "Plastic");

            //Position of frames in modules based on meshes and scale (0-point is between the two frames):
            float module_leftEdge = start.z + (f1.scale * leftBound.max.x);//x is z before they are rotated in the scene +
            float module_rightEdge = start.z + (f1.scale * rightBound.min.x);// -

            start.z = start.z - (module_leftEdge - module_rightEdge) - 0.01f;

            Simulation_Core.Joint j1 = DefineJoint(f1.guid, f2.guid, "Hinge", -(float)Math.PI / 2, (float)Math.PI / 2, 20.0f);

            Module module = DefineModule(f1, j1, f2);

            //CREATES A LOCK BETWEEN MODULES when adding new module to robot:
            if (i > 0)
            {
                robot_serialize.Add_Module(module, new Simulation_Core.Joint());
            }
            else
                robot_serialize.Add_Module(module/*,lockJoint*/);
        }

        //Assign object model names:
        robot_serialize.leftFrameDir = upperFrame_ObjName;
        robot_serialize.rightFrameDir = bottomFrame_ObjName;

        //CREATE Terrain:
        Texture2D hMap = Resources.Load("Heightmap3") as Texture2D;//Rename to terrain
        byte[] bytes = hMap.EncodeToPNG();

        var scene_serialize = DefineScene(bytes, Vector3.zero, "Rock", 10);

        //Add robot and scene to scenario:
        Scenario scenario_serialize = new Scenario()
        {
            robot = robot_serialize,
            scene = scene_serialize
        };

        //Add to XML file (SERIALIZE):
        Serialize(scenario_serialize);

    }

    public static void Serialize(object item)
    {
        string fileName = Application.streamingAssetsPath + "/XML/Scenario.xml";
        XmlSerializer serializer = new XmlSerializer(item.GetType());
        StreamWriter writer = new StreamWriter(fileName);
        serializer.Serialize(writer.BaseStream, item);
        writer.Close();
    }

    /*---------------------------------------------------Defining robot---------------------------------------------------*/
    /**--------------------------------------------------Adding module----------------------------------------------------*/
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

    /*-------------------------------------------------Helper Functions:--------------------------------------------------*/
    /*--------------------------------------------------Defining frames---------------------------------------------------*/
    Frame DefineFrame(string shape, UnityEngine.Vector3 pos, float scale, UnityEngine.Vector3 rot, float mass, string materialName)
    {
        return new Frame()//test create new object
        {
            guid = Guid.NewGuid(),
            shape = shape,
            position = pos,
            scale = scale,
            rotation = rot,
            mass = mass,
            isStatic = false,
            materialName = materialName
        };
    }
    Simulation_Core.Joint DefineJoint(Guid f1_guid, Guid f2_guid, string type, float l_rangeLimit, float r_rangeLimit, float max_vel)
    {
        return new Simulation_Core.Joint()
        {
            guid = Guid.NewGuid(),
            leftFrameGuid = f1_guid,
            rightFrameGuid = f2_guid,
            type = type,
            leftRangeLimit = l_rangeLimit,
            rightRangeLimit = r_rangeLimit,
            max_vel = max_vel

        };
    }
    Module DefineModule(Frame f1, Simulation_Core.Joint j, Frame f2)
    {
        var module = new Module();

        module.Create(f1, j, f2);

        return module;
    }
    Scene DefineScene(byte[] bytes, UnityEngine.Vector3 pos, string materialName, float height)
    {
        return new Scene()
        {
            guid = Guid.NewGuid(),
            height_Image = Convert.ToBase64String(bytes),
            position = new Vector3(),
            materialName = "Rock",
            height = 10
        };
    }
}
