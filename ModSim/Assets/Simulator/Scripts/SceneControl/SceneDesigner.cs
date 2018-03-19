using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Simulation_Core;
using System;
using System.Xml.Serialization;
using System.IO;
using UnityEditor;

public class SceneDesigner : MonoBehaviour {
    public GameObject SIMULATOR,CAMERA;

    public InputField left_scale, left_Position_X, left_Position_Y, left_Position_Z, left_Rotation_X, left_Rotation_Y, left_Rotation_Z, left_Material, left_Mass;

    public InputField right_scale, right_Position_X, right_Position_Y, right_Position_Z, right_Rotation_X, right_Rotation_Y, right_Rotation_Z, right_Material, right_Mass;

    public InputField leftFrame_name, rightFrame_name;

    public Text joint_jointType;
    public InputField joint_leftRangeLimit, joint_rightRangeLimit, joint_maxVelocity, joint_pValue;

    float l_scale, l_mass, r_scale, r_mass;
    Simulation_Core.Vector3 l_pos, l_rot, r_pos, r_rot;
    string l_mat, r_mat;

    string jointType;
    float leftRangeLimit, rightRangeLimit, maxVelocity, pValue;


    string dir = "";
    string upperFrame_ObjName = "upper.obj";
    string bottomFrame_ObjName = "bottom.obj";

    // Use this for initialization
    void Start ()
    {
        dir = Application.streamingAssetsPath + "/Robot/";//Get the path of the streaming assets
        upperFrame_ObjName = leftFrame_name.text;
        bottomFrame_ObjName = rightFrame_name.text;

        AddRobot();AddScene();
    }
	
	// Update is called once per frame
	void Update () {
		
	}
    /*----------------------------------------------Initializing Simulation-----------------------------------------------*/
    /**-----------------------------------------------Starting simulation-------------------------------------------------*/
    public void StartSimulation()
    {
        //Serialization();
        FinalizeCreation();
        SIMULATOR.SendMessage("Main_Initialization");
        CAMERA.SendMessage("Initialize");

    }
    /**------------------------------------------------Serializing robot-------------------------------------------------*/
    /*void Serialization()//Make into static class?
    {
        robot_serialize = new Robot();

        //Start position for the first module
        UnityEngine.Vector3 start_Position = new UnityEngine.Vector3(15, 12, 40);

        //For finding the size of the modules:
        ObjImporter import = new ObjImporter();

        Mesh leftMesh = import.ImportFile(dir + upperFrame_ObjName); Bounds leftBound = leftMesh.bounds;
        Mesh rightMesh = import.ImportFile(dir + bottomFrame_ObjName); Bounds rightBound = rightMesh.bounds;

        //Creating modules
        for (int i = 0; i < 10; i++)
        {
            //This, user should decide him/herself:
            var rot = i % 2 == 0 ? new UnityEngine.Vector3(0, -Mathf.PI / 2, 0) : new UnityEngine.Vector3(0, -Mathf.PI / 2, -Mathf.PI / 2);

            Frame f1 = DefineFrame("Box", start_Position, 10, rot, 50, "Plastic");
            Frame f2 = DefineFrame("Box", start_Position, 10, rot, 50, "Plastic");

            //Position of frames in modules based on meshes and scale (0-point is between the two frames):
            float module_leftEdge = start_Position.z + (f1.scale * leftBound.max.x);//x is z before they are rotated in the scene +
            float module_rightEdge = start_Position.z + (f1.scale * rightBound.min.x);// -

            start_Position.z = start_Position.z - (module_leftEdge - module_rightEdge) - 0.01f;

            Simulation_Core.Joint j1 = DefineJoint(f1.guid, f2.guid, "Hinge", -(float)Math.PI / 2, (float)Math.PI / 2, 20.0f);

            Module module = DefineModule(f1, j1, f2);

            //CREATES A LOCK BETWEEN MODULES when adding new module to robot:
            if (i > 0)
            {
                robot_serialize.Add_Module(module, new Simulation_Core.Joint());
            }
            else
                robot_serialize.Add_Module(module);
        }

        //Assign object model names:
        robot_serialize.leftFrameDir = upperFrame_ObjName;
        robot_serialize.rightFrameDir = bottomFrame_ObjName;

        //CREATE Terrain(ONCE):
        Texture2D hMap = Resources.Load("Heightmap3") as Texture2D;//Rename to terrain
        byte[] bytes = hMap.EncodeToPNG();

        var scene_serialize = DefineScene(bytes, UnityEngine.Vector3.zero, "Rock", 10);

        //Add robot and scene to scenario:
        Scenario scenario_serialize = new Scenario()
        {
            robot = robot_serialize,
            scene = scene_serialize
        };

        //Add to XML file (SERIALIZE):
        Serialize(scenario_serialize);

    }*/



    /*---------------------------------------------------Defining robot---------------------------------------------------*/

    Robot robot_serialize = new Robot();
    public void AddRobot()
    {
        robot_serialize = new Robot();

        List<Frame> frames = new List<Frame>();
        List<Simulation_Core.Joint> joints = new List<Simulation_Core.Joint>();

        //Assign object model names:
        robot_serialize.leftFrameDir = upperFrame_ObjName;
        robot_serialize.rightFrameDir = bottomFrame_ObjName;

    }
    Scene scene_serialize = new Scene();
    public void AddScene()
    {
        Texture2D hMap = Resources.Load("Heightmap3") as Texture2D;//Rename to terrain
        byte[] bytes = hMap.EncodeToPNG();

        scene_serialize = DefineScene(bytes, UnityEngine.Vector3.zero, "Rock", 10);
    }

    public void FinalizeCreation()
    {
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

    Bounds leftBound, rightBound;
    void RefreshBounds()
    {   //For finding the size of the modules:
        ObjImporter import = new ObjImporter();

        Mesh leftMesh = import.ImportFile(dir + upperFrame_ObjName); leftBound = leftMesh.bounds;
        Mesh rightMesh = import.ImportFile(dir + bottomFrame_ObjName); rightBound = rightMesh.bounds;
    }
    public void AddModules(int count)
    {
        RefreshBounds();
        //This, user should decide him/herself:
        var rot = count % 2 == 0 ? new UnityEngine.Vector3(0, -Mathf.PI / 2, 0) : new UnityEngine.Vector3(0, -Mathf.PI / 2, -Mathf.PI / 2);

        Frame f1 = DefineFrame("Box", l_pos, l_scale, l_rot, l_mass, l_mat);
        Frame f2 = DefineFrame("Box", l_pos, r_scale, r_rot, r_mass, r_mat);

        //Position of frames in modules based on meshes and scale (0-point is between the two frames):
        float module_leftEdge = (float)currentModulePosition.z + (f1.scale * leftBound.max.x);//x is z before they are rotated in the scene +
        float module_rightEdge = (float)currentModulePosition.z + (f1.scale * rightBound.min.x);// -

        //if next is a module:
        currentModulePosition.z = currentModulePosition.z - (module_leftEdge - module_rightEdge) - 0.01f;

        Simulation_Core.Joint j1 = DefineJoint(f1.guid, f2.guid, jointType, leftRangeLimit, rightRangeLimit, maxVelocity);

        Module module = DefineModule(f1, j1, f2);
        //Which number, for the sensor modules:
        module.mod_Nr = count;

        //CREATES A LOCK BETWEEN MODULES when adding new module to robot:
        if (count > 0)
        {
            robot_serialize.Add_Module(module, new Simulation_Core.Joint());
        }
        else
            robot_serialize.Add_Module(module);

    }
    /**--------------------------------------------------Adding module----------------------------------------------------*/
    int module_Count = 0;
    Simulation_Core.Vector3 currentModulePosition = new Simulation_Core.Vector3(15, 12, 40);
    bool FirstModule = true;
    public void ButtonAddModule()
    {
        
        GetFrameValues();
        GetJointValues();

        //Only once:
        if (FirstModule)
        {
            currentModulePosition = l_pos;
            FirstModule = false;
        }

        

        AddModules(module_Count);module_Count++;

        PrepareForNextInput();
        Debug.Log("Module created: ");

    }

    void PrepareForNextInput()
    {
        PrepareNextCurrentPosition();

        //Y should always be rotated, atleast propose it:
        left_Rotation_Y.text = right_Rotation_Y.text = "-90";

        if (module_Count % 2 != 0)
            left_Rotation_Z.text = right_Rotation_Z.text ="90";
        else
            left_Rotation_Z.text = right_Rotation_Z.text = "0";
    }
    void PrepareNextCurrentPosition()
    {
        left_Position_X.text = currentModulePosition.x.ToString();
        left_Position_Y.text = currentModulePosition.y.ToString();
        left_Position_Z.text = currentModulePosition.z.ToString();

        right_Position_X.text = currentModulePosition.x.ToString();
        right_Position_Y.text = currentModulePosition.y.ToString();
        right_Position_Z.text = currentModulePosition.z.ToString();
    }

    public InputField Ism_leftnr, Ism_rightnr, Ism_pos_x, Ism_pos_y, Ism_pos_z, Ism_size_x, Ism_size_y, Ism_size_z, Ism_mat, Ism_mass;
    Simulation_Core.Vector3 sm_pos, sm_size;
    float sm_mass;
    string sm_mat;

    public GameObject sensoryModulePanel;
    bool sm_panelOpen = false;

    public Text addSensoryModuleText;

    public void ButtonAddSensoryModule()
    {
        //Opens and closes the sensory module panel when needed:
        if(sm_panelOpen)
        {//Closes panel
            AddSensoryModule();
            sensoryModulePanel.gameObject.SetActive(false);
            addSensoryModuleText.text = "Add Sensor Module";
            sm_panelOpen = false;
        }else
        {//Opens panel
            PrepareSMPosition();
            sensoryModulePanel.gameObject.SetActive(true);
            addSensoryModuleText.text = "Create";
            sm_panelOpen = true;
        }
    }

    public void AddSensoryModule()
    {
        if (GetSensoryModuleValues())
        {

            bool first_ModulePos = false;
            //Get values just to estimate the frame params. Thus, we can calculate the next position of currentModulePosition.
            GetFrameValues(); GetJointValues();

            //Ensure the current mesh is calculated:
            RefreshBounds();

            var leftModSize_Z = (l_scale * ((leftBound.max.x + Math.Abs(rightBound.min.x))/2));//length of the left module

            if (FirstModule)
            {
                //Position for this module center:
                currentModulePosition = l_pos;
                FirstModule = false;
                first_ModulePos = true;
            }
            else//From left, position:
            {
                //Position for this module center, if not first:
                currentModulePosition.z = currentModulePosition.z + leftModSize_Z - sm_size.z - 0.01f;//Sensor.z = sensor.z + (l_scale * leftSize.z) - sensorScale.z/2 (+ because we are going backwards)
            }

            //Create the sensory module:
            var mod = DefineSensoryModule(module_Count - 1, module_Count, currentModulePosition, sm_size, sm_mass, Simulation_Core.Vector3.zero, sm_mat);//module count-1 and module count will be the place where sensor module is set.
            Debug.Log("Left: " + (module_Count - 1) + ", Right: " + module_Count);
            if (first_ModulePos)
            {
                robot_serialize.Add_SensorModule(mod, new Simulation_Core.Joint());
            }
            else
                robot_serialize.Add_SensorModule(new Simulation_Core.Joint(), mod, new Simulation_Core.Joint());

            //Position for the next module center:
            currentModulePosition.z = currentModulePosition.z - sm_size.z  - leftModSize_Z - 0.01f;// currentModulePosition.z - (module_leftEdge - module_rightEdge) - 0.01f;

            PrepareNextCurrentPosition();
        }
    }

    void PrepareSMPosition()
    {
        Ism_pos_x.text = currentModulePosition.x.ToString();
        Ism_pos_y.text = currentModulePosition.y.ToString();
        Ism_pos_z.text = currentModulePosition.z.ToString();
        Ism_leftnr.text = (module_Count - 1).ToString();
        Ism_rightnr.text = module_Count.ToString();
    }
    
    public void RemoveModule()
    {

    }

    void GetFrameValues()
    {
        //Left Frame:
        float.TryParse(left_scale.text, out l_scale);
        double.TryParse(left_Position_X.text, out l_pos.x);
        double.TryParse(left_Position_Y.text, out l_pos.y);
        double.TryParse(left_Position_Z.text, out l_pos.z);

        double.TryParse(left_Rotation_X.text, out l_rot.x);l_rot.x *= ((float)Math.PI / 180);//deg to rad
        double.TryParse(left_Rotation_Y.text, out l_rot.y);l_rot.y *= ((float)Math.PI / 180);//deg to rad
        double.TryParse(left_Rotation_Z.text, out l_rot.z);l_rot.z *= ((float)Math.PI / 180);//deg to rad
        l_mat = left_Material.text;
        float.TryParse(left_Mass.text, out l_mass);

        //Right Frame:
        float.TryParse(right_scale.text, out r_scale);
        double.TryParse(right_Position_X.text, out r_pos.x);
        double.TryParse(right_Position_Y.text, out r_pos.y);
        double.TryParse(right_Position_Z.text, out r_pos.z);
        double.TryParse(right_Rotation_X.text, out r_rot.x); r_rot.x *= ((float)Math.PI / 180);//deg to rad
        double.TryParse(right_Rotation_Y.text, out r_rot.y); r_rot.y *= ((float)Math.PI / 180);//deg to rad
        double.TryParse(right_Rotation_Z.text, out r_rot.z); r_rot.z *= ((float)Math.PI / 180);//deg to rad
        r_mat = right_Material.text;
        float.TryParse(right_Mass.text, out l_mass);

        Debug.Log("Frames created");
    }

    void GetJointValues()
    {
        jointType = joint_jointType.text;
        float.TryParse(joint_leftRangeLimit.text, out leftRangeLimit);leftRangeLimit *= ((float)Math.PI/180);//deg to rad
        float.TryParse(joint_rightRangeLimit.text, out rightRangeLimit); rightRangeLimit *= ((float)Math.PI/180);//deg to rad
        float.TryParse(joint_maxVelocity.text, out maxVelocity);
        float.TryParse(joint_pValue.text, out pValue);

        Debug.Log("Joint created");
    }

    bool GetSensoryModuleValues()
    {
        try
        {
            double.TryParse(Ism_pos_x.text, out sm_pos.x);
            double.TryParse(Ism_pos_y.text, out sm_pos.y);
            double.TryParse(Ism_pos_z.text, out sm_pos.z);

            double.TryParse(Ism_size_x.text, out sm_size.x);
            double.TryParse(Ism_size_y.text, out sm_size.y);
            double.TryParse(Ism_size_z.text, out sm_size.z);

            sm_mat = Ism_mat.text;
            float.TryParse(Ism_mass.text, out sm_mass);
            return true;
        }catch(NullReferenceException)
        {
            Debug.Log("could not get textbox data");
            return false;
        }

    }

    /*-----------------------------------------------------Settings-------------------------------------------------------*/
    /*---------------------------------------------------Save Config:-----------------------------------------------------*/
    public void SaveConfig()
    {

    }

    public void LoadConfig()
    {
    }
    /*------------------------------------------------Hiding the designer-------------------------------------------------*/
    public GameObject mainPanel;
    public void ToggleDesigner(bool toggled)
    {
        if (toggled)
            mainPanel.gameObject.SetActive(true);
        else
            mainPanel.gameObject.SetActive(false);
    }

    /*-------------------------------------------------Helper Functions:--------------------------------------------------*/
    /*--------------------------------------------------Defining frames---------------------------------------------------*/
    Frame DefineFrame(string shape, Simulation_Core.Vector3 pos, float scale, Simulation_Core.Vector3 rot, float mass, string materialName)
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
            position = new Simulation_Core.Vector3(),
            materialName = "Rock",
            height = 10
        };
    }
    Sensory_Module DefineSensoryModule(int leftModNr, int rightModNr, Simulation_Core.Vector3 pos, Simulation_Core.Vector3 size, float mass, Simulation_Core.Vector3 rot, string materialName )
    {
        return new Sensory_Module()
        {
            guid = Guid.NewGuid(),
            leftMod_Nr = leftModNr,
            rightMod_Nr = rightModNr, //if left is -1, it is the first module. if right is -1, it is the last module.
            position = pos,
            size = size,
            materialName = materialName,
            mass = mass,
            rotation = rot
        };
    }
}
