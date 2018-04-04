/* Scene Designer, simulation preparation class (Scene_Designer)
 * Torstein Sundnes Lenerand
 * NTNU Ålesund
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Simulation_Core;
using System;
using System.Xml.Serialization;
using System.IO;
using UnityEditor;

public class Scene_Designer : MonoBehaviour {
    public GameObject SIMULATOR,CAMERA;

    public InputField left_scale, left_Position_X, left_Position_Y, left_Position_Z, left_Rotation_X, left_Rotation_Y, left_Rotation_Z, left_Material, left_Mass;

    public InputField right_scale, right_Position_X, right_Position_Y, right_Position_Z, right_Rotation_X, right_Rotation_Y, right_Rotation_Z, right_Material, right_Mass;

    public InputField leftFrame_name, rightFrame_name;

    public Text joint_jointType;
    public InputField joint_leftRangeLimit, joint_rightRangeLimit, joint_maxVelocity, joint_pValue;

    double l_scale, l_mass, r_scale, r_mass;
    AgX_Interface.Vector3 l_pos, l_rot, r_pos, r_rot;
    string l_mat, r_mat;

    string jointType;
    double leftRangeLimit, rightRangeLimit, maxVelocity, pValue;

    string dir = "";
    string upperFrame_ObjName = "upper.obj";
    string bottomFrame_ObjName = "bottom.obj";

    // Use this for initialization
    void Start ()
    {
        dir = Application.streamingAssetsPath + "/Robot/";//Get the path of the streaming assets
        upperFrame_ObjName = leftFrame_name.text;
        bottomFrame_ObjName = rightFrame_name.text;

        AddRobot();
        VisualizeScene();
    }
	
	// Update is called once per frame
	void Update () {
		
	}
    /*----------------------------------------------Initializing Simulation-----------------------------------------------*/
    /**-----------------------------------------------Starting simulation-------------------------------------------------*/
    bool StartDesigner = false;
    
    public void StartSimulation()
    {
        if (StartDesigner == false)
        {
            //Serialization();
            //Scene:
            AddScene();

            //MAKE INTO SEPARATE FUNCTION(also the other place it is used):
            for (int i = 0; i < current_frameVis.Count; i++)
            {
                current_frameVis[i].Remove();
                //frameVis.Remove(frameVis[i]);
            }
            current_frameVis.Clear();

            for (int i = 0; i < current_SMVis.Count; ++i)
            {
                current_SMVis[i].Remove();
            }
            current_SMVis.Clear();


            if (SM_planned != null)
                if (SM_planned.gameobject.gameObject != null)
                {
                    SM_planned.Remove(); SM_planned = null;
                }

            if (left_planned != null)
                if (left_planned.gameobject.gameObject != null)
                {
                    left_planned.Remove(); left_planned = null;
                }
            if (right_planned != null)
                if (right_planned.gameobject.gameObject != null)
                {
                    right_planned.Remove(); right_planned = null;
                }

            RemoveSceneVis();

            //Serializing Robot:
            FinalizeCreation();
            SetAnalyticsPath();
            UpdateSimTime();
            UpdateOptimizationTime();
            SIMULATOR.SendMessage("Main_Initialization");
            CAMERA.SendMessage("Initialize");
            StartDesigner = true;
        }
        else
        {
            SIMULATOR.SendMessage("Stop");
            StartDesigner = false;
        }

    }
    public InputField deltaTime;
    public void UpdateSimTime()
    {
        float dt;
        float.TryParse(deltaTime.text, out dt);
        SIMULATOR.SendMessage("ChangeDeltaTime", dt);
    }
    public InputField optimizationTime;
    public void UpdateOptimizationTime()
    {
        float dt;
        float.TryParse(optimizationTime.text, out dt);

        //Update time in main
        SIMULATOR.SendMessage("UpdateRepeatRate",dt);
    }
    public void StopSimulation()
    {
        SIMULATOR.SendMessage("Stop");
        StartDesigner = false;
    }

    public void PauseSimulation()
    {
        SIMULATOR.SendMessage("Pause");
    }

    public InputField analyticsPathName,analyticsFileName,Interval;
    void SetAnalyticsPath()
    {
        Analytics_Visualization.Input_Filename = analyticsPathName.text + @"\" + analyticsFileName.text;
        double.TryParse(Interval.text, out Analytics_Visualization.Interval);
    }

    public void ClearFileButton()
    {
        string path = analyticsPathName.text + @"\" + analyticsFileName.text;
        if (File.Exists(path))
            File.Delete(path);
    }
    /**------------------------------------------------Serializing robot-------------------------------------------------*/
    /*void Serialization()//Make into static class?
    {
        robot_serialize = new Robot();

        //Start position for the first module
        .Vector3 start_Position = new .Vector3(15, 12, 40);

        //For finding the size of the modules:
        ObjImporter import = new ObjImporter();

        Mesh leftMesh = import.ImportFile(dir + upperFrame_ObjName); Bounds leftBound = leftMesh.bounds;
        Mesh rightMesh = import.ImportFile(dir + bottomFrame_ObjName); Bounds rightBound = rightMesh.bounds;

        //Creating modules
        for (int i = 0; i < 10; i++)
        {
            //This, user should decide him/herself:
            var rot = i % 2 == 0 ? new .Vector3(0, -Mathf.PI / 2, 0) : new .Vector3(0, -Mathf.PI / 2, -Mathf.PI / 2);

            Frame f1 = DefineFrame("Box", start_Position, 10, rot, 50, "Plastic");
            Frame f2 = DefineFrame("Box", start_Position, 10, rot, 50, "Plastic");

            //Position of frames in modules based on meshes and scale (0-point is between the two frames):
            double module_leftEdge = start_Position.z + (f1.scale * leftBound.max.x);//x is z before they are rotated in the scene +
            double module_rightEdge = start_Position.z + (f1.scale * rightBound.min.x);// -

            start_Position.z = start_Position.z - (module_leftEdge - module_rightEdge) - 0.01f;

            Simulation_Core.Joint j1 = DefineJoint(f1.guid, f2.guid, "Hinge", -(double)Math.PI / 2, (double)Math.PI / 2, 20.0f);

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

        var scene_serialize = DefineScene(bytes, .Vector3.zero, "Rock", 10);

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

    Robot robot_ForSerialization = new Robot();
    public void AddRobot()
    {
        robot_ForSerialization = new Robot();

        List<Frame> frames = new List<Frame>();
        List<Simulation_Core.Joint> joints = new List<Simulation_Core.Joint>();

        //Assign object model names:
        robot_ForSerialization.leftFrameDir = upperFrame_ObjName;
        robot_ForSerialization.rightFrameDir = bottomFrame_ObjName;

    }
    public InputField HeightMapSelection;
    Scene scene_serialize = new Scene();
    public void AddScene()
    {
        Texture2D hMap = previewImage.GetComponent<RawImage>().texture as Texture2D;
        //Texture2D hMap = Resources.Load(HeightMapSelection.text) as Texture2D;//Rename to terrain
        byte[] bytes = hMap.EncodeToPNG();

        scene_serialize = DefineScene(bytes, new AgX_Interface.Vector3(-125,0,-125), "Rock", 10);
    }

    public void FinalizeCreation()
    {
        //Add robot and scene to scenario:
        Scenario scenario_serialize = new Scenario()
        {
            robot = robot_ForSerialization,
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
        //  var rot = count % 2 == 0 ? new .Vector3(0, -Mathf.PI / 2, 0) : new .Vector3(0, -Mathf.PI / 2, -Mathf.PI / 2);
        var l_q = Quaternion.Euler((float)l_rot.x, (float)l_rot.y, (float)l_rot.z);
        var r_q = Quaternion.Euler((float)r_rot.x, (float)r_rot.y, (float)r_rot.z);

        AgX_Interface.Quaternion l_quat = new AgX_Interface.Quaternion(l_q.x, l_q.y, l_q.z, l_q.w);
        AgX_Interface.Quaternion r_quat = new AgX_Interface.Quaternion(r_q.x, r_q.y, r_q.z, r_q.w);


        Frame f1 = DefineFrame("Box", l_pos, l_scale, l_quat, l_mass, l_mat);
        Frame f2 = DefineFrame("Box", r_pos, r_scale, r_quat, r_mass, r_mat);

        //Position of frames in modules based on meshes and scale (0-point is between the two frames):
        double module_leftEdge = (double)currentModulePosition.z + (f1.scale * leftBound.max.x);//x is z before they are rotated in the scene +
        double module_rightEdge = (double)currentModulePosition.z + (f1.scale * rightBound.min.x);// -

        //if next is a module:
        currentModulePosition.z = currentModulePosition.z - (module_leftEdge - module_rightEdge) - 0.01f;

        Simulation_Core.Joint j1 = DefineJoint(f1.guid, f2.guid, jointType, leftRangeLimit, rightRangeLimit, maxVelocity);

        Module module = DefineModule(f1, j1, f2);
        //Which number, for the sensor modules:
        module.mod_Nr = count;

        //CREATES A LOCK BETWEEN MODULES when adding new module to robot:
        if (module_Count == 0 && SensoryModule_Count == 0)
        {
            robot_ForSerialization.Add_Module(module);
            
        }
        else
            robot_ForSerialization.Add_Module(module, new Simulation_Core.Joint());

    }
    /**--------------------------------------------------Adding module----------------------------------------------------*/
    int module_Count = 0;
    AgX_Interface.Vector3 currentModulePosition = AgX_Interface.Vector3.zero;
    bool FirstModule = true;
    public InputField moduleCount;
    public void ButtonAddModule()
    {
        
        GetFrameValues();
        GetJointValues();

        //Only once:
        if (FirstModule)
        {
            currentModulePosition = r_pos = l_pos;
            FirstModule = false;
        }

        

        AddModules(module_Count);module_Count++;

        moduleCount.text = module_Count.ToString();

        PrepareForNextModuleInput();
        Debug.Log("Module created: ");

    }

    void PrepareForNextModuleInput()
    {
        PrepareNextCurrentPosition();

        //Y should always be rotated, atleast propose it:
        left_Rotation_Y.text = right_Rotation_Y.text = "-90";l_rot.y = r_rot.y = -90;

        if (module_Count % 2 != 0)
        {
            left_Rotation_X.text = right_Rotation_X.text = "90";l_rot.x = r_rot.x = 90;
        }
        else
        {
            left_Rotation_X.text = right_Rotation_X.text = "0";l_rot.x = r_rot.x = 0;
        }

        ShowCurrentRobotConfig();
        //Show the robot that will be created if current values are selected:(RUN ONCE)
        ShowPlannedModuleConfig();
    }
    void PrepareNextCurrentPosition()
    {
        left_Position_X.text = currentModulePosition.x.ToString();
        left_Position_Y.text = currentModulePosition.y.ToString();
        left_Position_Z.text = currentModulePosition.z.ToString();

        right_Position_X.text = currentModulePosition.x.ToString();
        right_Position_Y.text = currentModulePosition.y.ToString();
        right_Position_Z.text = currentModulePosition.z.ToString();

        //Show the robot thus far:
        ShowCurrentRobotConfig();
        //Show the robot that will be created if current values are selected:(RUN ONCE)
        ShowPlannedModuleConfig();
    }

    public InputField Ism_leftnr, Ism_rightnr, Ism_pos_x, Ism_pos_y, Ism_pos_z, Ism_size_x, Ism_size_y, Ism_size_z, Ism_mat, Ism_mass;
    AgX_Interface.Vector3 sm_pos, sm_size;
    double sm_mass;
    string sm_mat;

    public GameObject sensoryModulePanel;
    bool sm_panelOpen = false;

    public Text addSensoryModuleText;
    public InputField sensorModuleCount;

    int SensoryModule_Count = 0;

    public void ButtonAddSensoryModule()
    {
        //Opens and closes the sensory module panel when needed:
        if(sm_panelOpen)
        {//Closes panel
            if (AddSensoryModule())
                SensoryModule_Count++;
            sensoryModulePanel.gameObject.SetActive(false);
            addSensoryModuleText.text = "Add Sensor Module";
            sm_panelOpen = false;
            sensorModuleCount.text = SensoryModule_Count.ToString();
        }else
        {//Opens panel
            PrepareSMPosition();
            sensoryModulePanel.gameObject.SetActive(true);
            addSensoryModuleText.text = "Create";
            sm_panelOpen = true;
        }
    }

    bool AddSensoryModule()
    {
        if (GetSensoryModuleValues())
        {

            //Get values just to estimate the frame params. Thus, we can calculate the next position of currentModulePosition.
            GetFrameValues(); GetJointValues();

            //Ensure the current mesh is calculated:
            RefreshBounds();

            var leftModSize_Z = (l_scale * ((leftBound.max.x + Math.Abs(rightBound.min.x))/2));//length of the left module

            if (module_Count == 0 && SensoryModule_Count == 0)
            {
                //Position for this module center:
                currentModulePosition = r_pos = l_pos;
                FirstModule = false;
            }
            else//From left, position:
            {
                //Position for this module center, if not first:
                currentModulePosition.z = currentModulePosition.z + leftModSize_Z - sm_size.z - 0.01f;//Sensor.z = sensor.z + (l_scale * leftSize.z) - sensorScale.z/2 (+ because we are going backwards)
            }

            //Create the sensory module:
            var mod = DefineSensoryModule(module_Count - 1, module_Count, currentModulePosition, sm_size, sm_mass, new AgX_Interface.Quaternion(0,0,0,1), sm_mat);//module count-1 and module count will be the place where sensor module is set.
            Debug.Log("Left: " + (module_Count - 1) + ", Right: " + module_Count);
            if (module_Count == 0 && SensoryModule_Count == 0)
            {
                robot_ForSerialization.Add_SensorModule(mod, new Simulation_Core.Joint());
            }
            else
                robot_ForSerialization.Add_SensorModule(new Simulation_Core.Joint(), mod, new Simulation_Core.Joint());

            //Position for the next module center:
            currentModulePosition.z = currentModulePosition.z - sm_size.z  - leftModSize_Z - 0.01f;// currentModulePosition.z - (module_leftEdge - module_rightEdge) - 0.01f;

            PrepareNextCurrentPosition();
            return true;
        }
        return false;
    }

    void PrepareSMPosition()
    {
        if (module_Count == 0 && SensoryModule_Count == 0)
        {
            Ism_pos_x.text = left_Position_X.text.ToString();
            Ism_pos_y.text = left_Position_Y.text.ToString();
            Ism_pos_z.text = left_Position_Z.text.ToString();
        }
        else
        {
            Ism_pos_x.text = currentModulePosition.x.ToString();
            Ism_pos_y.text = currentModulePosition.y.ToString();
            Ism_pos_z.text = currentModulePosition.z.ToString();
        }
        Ism_leftnr.text = (module_Count - 1).ToString();
        Ism_rightnr.text = module_Count.ToString();
    }
    
    public void RemoveModule()
    {

    }

    void GetFrameValues()
    {
        //Left Frame:
        double.TryParse(left_scale.text, out l_scale);
        double.TryParse(left_Position_X.text, out l_pos.x);
        double.TryParse(left_Position_Y.text, out l_pos.y);
        double.TryParse(left_Position_Z.text, out l_pos.z);

        double.TryParse(left_Rotation_X.text, out l_rot.x);//l_rot.x *= (Math.PI / 180);//deg to rad
        double.TryParse(left_Rotation_Y.text, out l_rot.y);//l_rot.y *= (Math.PI / 180);//deg to rad
        double.TryParse(left_Rotation_Z.text, out l_rot.z);//l_rot.z *= (Math.PI / 180);//deg to rad
        l_mat = left_Material.text;
        double.TryParse(left_Mass.text, out l_mass);

        //Right Frame:
        double.TryParse(right_scale.text, out r_scale);
        double.TryParse(right_Position_X.text, out r_pos.x);
        double.TryParse(right_Position_Y.text, out r_pos.y);
        double.TryParse(right_Position_Z.text, out r_pos.z);
        double.TryParse(right_Rotation_X.text, out r_rot.x);// r_rot.x *= (Math.PI / 180);//deg to rad
        double.TryParse(right_Rotation_Y.text, out r_rot.y);// r_rot.y *= (Math.PI / 180);//deg to rad
        double.TryParse(right_Rotation_Z.text, out r_rot.z);// r_rot.z *= (Math.PI / 180);//deg to rad
        r_mat = right_Material.text;
        double.TryParse(right_Mass.text, out r_mass);

        Debug.Log("Frames created");
    }

    void GetJointValues()
    {
        jointType = joint_jointType.text;
        double.TryParse(joint_leftRangeLimit.text, out leftRangeLimit);leftRangeLimit *= ((double)Math.PI/180);//deg to rad
        double.TryParse(joint_rightRangeLimit.text, out rightRangeLimit); rightRangeLimit *= ((double)Math.PI/180);//deg to rad
        double.TryParse(joint_maxVelocity.text, out maxVelocity);
        double.TryParse(joint_pValue.text, out pValue);

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
            double.TryParse(Ism_mass.text, out sm_mass);
            return true;
        }catch(NullReferenceException)
        {
            Debug.Log("could not get textbox data");
            return false;
        }

    }

    /*--------------------------------------------------Visualizations----------------------------------------------------*/
    /*--------------------------------------------Visualizing current modules---------------------------------------------*/
    List<Unity_Visualization.Frame_Vis> current_frameVis = new List<Unity_Visualization.Frame_Vis>();
    List<Unity_Visualization.Sensor_Vis> current_SMVis = new List<Unity_Visualization.Sensor_Vis>();
    void ShowCurrentRobotConfig()
    {
        ObjImporter import = new ObjImporter();
        
        //Frame vis reset:
        for(int i = 0; i<current_frameVis.Count; i++)
        {
            current_frameVis[i].Remove();
            //frameVis.Remove(frameVis[i]);
        }
        current_frameVis.Clear();

        //Sensory module vis reset:
        for(int i = 0; i<current_SMVis.Count; ++i)
        {
            current_SMVis[i].Remove();
        }
        current_SMVis.Clear();

        foreach (Module mod in robot_ForSerialization.modules)
        {   Mesh leftMesh = import.ImportFile(dir + robot_ForSerialization.leftFrameDir);
            Mesh rightMesh = import.ImportFile(dir + robot_ForSerialization.rightFrameDir);

            current_frameVis.Add(new Unity_Visualization.Frame_Vis(mod.frames[0].guid, leftMesh ,new Vector3((float)mod.frames[0].position.x,(float)mod.frames[0].position.y,(float)mod.frames[0].position.z),mod.frames[0].scale));
            current_frameVis.Add(new Unity_Visualization.Frame_Vis(mod.frames[1].guid, rightMesh, new Vector3((float)mod.frames[1].position.x, (float)mod.frames[1].position.y, (float)mod.frames[1].position.z), mod.frames[1].scale));
            //Debug.Log("Vert: " + leftMesh.vertices[5] + " |Scale: " + mod.frames[0].scale);
            //Update:
            foreach(Frame frame in mod.frames)
            try { current_frameVis.Find(x => x.guid == frame.guid).Update(
                new Vector3((float)frame.position.x,(float)frame.position.y,(float)frame.position.z), 
                new Quaternion((float)frame.quatRotation.x,(float)frame.quatRotation.y,(float)frame.quatRotation.z,(float)frame.quatRotation.w), //needs to be in degrees for vis
                mod.Axis);
                }
            catch (NullReferenceException e) { Debug.Log("Could not create frame gameobject."); }
        }

        //SensoryModules:
        foreach(Sensor_Module mod in robot_ForSerialization.sensorModules)
        {
            current_SMVis.Add(new Unity_Visualization.Sensor_Vis(mod.guid,new Vector3((float)mod.position.x,(float)mod.position.y,(float)mod.position.z),new Vector3((float)mod.size.x,(float)mod.size.y,(float)mod.size.z)));

        }

        //Debug.Log(current_frameVis.Count);

        //Update:
    }
    /*----------------------------------------------Visualizing next module-----------------------------------------------*/
    //List<Unity_Visualization.Frame_Vis> next_frameVis = new List<Unity_Visualization.Frame_Vis>();
    Unity_Visualization.Frame_Vis left_planned, right_planned;
    public void ShowPlannedModuleConfig()//Called each time an input field is changed
    {
        if (SM_planned != null)
            if (SM_planned.gameobject.gameObject != null)
            {
                SM_planned.Remove();SM_planned = null;
            }

        GetFrameValues();
        ObjImporter import = new ObjImporter();
        //if one of the InputFields of frames or joint is changed
        //create gameobject with mesh n stuff
        Mesh leftMesh = import.ImportFile(dir + robot_ForSerialization.leftFrameDir);
        Mesh rightMesh = import.ImportFile(dir + robot_ForSerialization.rightFrameDir);
        if (left_planned == null && right_planned == null)
        {
            left_planned = new Unity_Visualization.Frame_Vis(Guid.NewGuid(), leftMesh, new Vector3((float)l_pos.x, (float)l_pos.y, (float)l_pos.z), l_scale);
            right_planned = new Unity_Visualization.Frame_Vis(Guid.NewGuid(), rightMesh, new Vector3((float)r_pos.x, (float)r_pos.y, (float)r_pos.z), r_scale);

            left_planned.Update(new Vector3((float)l_pos.x, (float)l_pos.y, (float)l_pos.z), new Vector3((float)l_rot.x, (float)l_rot.y, (float)l_rot.z), l_rot.x == 0 ? "Pitch" : "Yaw");
            right_planned.Update(new Vector3((float)r_pos.x, (float)r_pos.y, (float)r_pos.z), new Vector3((float)r_rot.x, (float)r_rot.y, (float)r_rot.z), r_rot.x == 0 ? "Pitch" : "Yaw");
        }
        else
        {
            left_planned.Update(new Vector3((float)l_pos.x, (float)l_pos.y, (float)l_pos.z),new Vector3((float)l_rot.x,(float)l_rot.y,(float)l_rot.z), l_rot.x == 0 ? "Pitch" : "Yaw");
            right_planned.Update(new Vector3((float)r_pos.x, (float)r_pos.y, (float)r_pos.z), new Vector3((float)r_rot.x, (float)r_rot.y, (float)r_rot.z), r_rot.x == 0 ? "Pitch" : "Yaw");
        }
    }

    Unity_Visualization.Sensor_Vis SM_planned;
    public void ShowPlannedSensoryModuleConfig()
    {
        //sm_pos = new AgX_Interface.Vector3((float)l_pos.x, (float)l_pos.y, (float)l_pos.z);
        if (left_planned != null)
            if (left_planned.gameobject.gameObject != null)
            {
                left_planned.Remove();left_planned = null;
            }
        if (right_planned != null)
            if (right_planned.gameobject.gameObject != null)
            {
                right_planned.Remove();right_planned = null;
            }

        GetSensoryModuleValues();

        if (SM_planned == null)
        {
            SM_planned = new Unity_Visualization.Sensor_Vis(Guid.NewGuid(), new Vector3((float)sm_pos.x, (float)sm_pos.y, (float)sm_pos.z), new Vector3((float)sm_size.x, (float)sm_size.y, (float)sm_size.z));
            SM_planned.Update(new Vector3((float)sm_pos.x, (float)sm_pos.y, (float)sm_pos.z), Vector3.zero, new Vector3((float)sm_size.x, (float)sm_size.y, (float)sm_size.z));
        } else
        {
            SM_planned.Update(new Vector3((float)sm_pos.x, (float)sm_pos.y, (float)sm_pos.z), Vector3.zero, new Vector3((float)sm_size.x, (float)sm_size.y, (float)sm_size.z));Debug.Log("working");
        }
    }

    /*--------------------------------------------------Visualize Scene---------------------------------------------------*/
    /*------------------------------------------------Opening file dialog-------------------------------------------------*/
    public class OpenTerrain : EditorWindow
    {
        public static string terrainPath = "";
        [MenuItem("Example/Overwrite Texture")]
        public static void LOADTerrain()
        {
            terrainPath = EditorUtility.OpenFilePanel("Upload a PNG file", Application.streamingAssetsPath, "png");
            Debug.Log(terrainPath);

        }
    }
    public void LoadTerrainButton()
    {
        OpenTerrain.LOADTerrain();
        VisualizeScene();
    }
    Unity_Visualization.Scene_Vis sceneVis;
    Scene sceneVals;
    public GameObject previewImage;
    public void VisualizeScene()
    {
        try
        {
            Texture2D hMap = Resources.Load("terrain") as Texture2D;//Rename to terrain

            if (OpenTerrain.terrainPath != "")//if path i set by file browser: upload png.
            {
                var fileContent = File.ReadAllBytes(OpenTerrain.terrainPath);
                hMap.LoadImage(fileContent);
                HeightMapSelection.text = OpenTerrain.terrainPath;
            }

            RawImage img = previewImage.GetComponent<RawImage>();
            img.texture = hMap;

            byte[] bytes = hMap.EncodeToPNG();

            //scene visualization
            sceneVals = DefineScene(bytes, new AgX_Interface.Vector3(-125, 0, -125), "Rock", 10);
            sceneVals.CreateMesh();

            if (sceneVis == null)//first time scene is loaded
            {
                sceneVis = new Unity_Visualization.Scene_Vis(sceneVals.guid, AgxHelper(sceneVals.vertices), sceneVals.triangles, AgxHelper(sceneVals.uvs), new UnityEngine.Vector3((float)sceneVals.position.x, (float)sceneVals.position.y, (float)sceneVals.position.z), Resources.Load("grass") as Texture);
            }
            else
            {
                sceneVis.Remove();
                sceneVis = new Unity_Visualization.Scene_Vis(sceneVals.guid, AgxHelper(sceneVals.vertices), sceneVals.triangles, AgxHelper(sceneVals.uvs), new UnityEngine.Vector3((float)sceneVals.position.x, (float)sceneVals.position.y, (float)sceneVals.position.z), Resources.Load("grass") as Texture);
            }
        }
        catch (NullReferenceException e)
        {
            //PopupWindow.Show(new Rect(0,0,500,500),);
        }
    }
    void RemoveSceneVis()
    {
        if(sceneVis != null)
        {
            sceneVis.Remove();
            sceneVis = null;
        }
    }
    public InputField start_Position_X, start_Position_Y, start_Position_Z;
    AgX_Interface.Vector3 old_robot_pos, new_robot_pos;
    public void RobotPositionChanged()
    {
        double.TryParse(start_Position_X.text, out new_robot_pos.x);
        double.TryParse(start_Position_Y.text, out new_robot_pos.y);
        double.TryParse(start_Position_Z.text, out new_robot_pos.z);

        var distance = ModifyRobotPosition(robot_ForSerialization, new_robot_pos);

        l_pos += distance;
        r_pos += distance;
        sm_pos += distance;
        currentModulePosition += distance;

        if(left_planned != null && right_planned != null)
        left_planned.gameobject.transform.position = right_planned.gameobject.transform.position = new Vector3((float)currentModulePosition.x,(float)currentModulePosition.y,(float)currentModulePosition.x);

        if (SM_planned != null)
            SM_planned.gameobject.transform.position = new Vector3((float)currentModulePosition.x, (float)currentModulePosition.y, (float)currentModulePosition.z);

        UpdateUIText();

        ShowCurrentRobotConfig();

    }
    AgX_Interface.Vector3 prevPos = new AgX_Interface.Vector3(0,12,0), origin = AgX_Interface.Vector3.zero;
    bool first = true;
    AgX_Interface.Vector3 ModifyRobotPosition(Robot robot, AgX_Interface.Vector3 new_pos)
    {

        var midDistance = origin - prevPos;
        midDistance += new_pos;  


        foreach(Module mod in robot_ForSerialization.modules)
        {
            mod.frames[0].position += midDistance;
            mod.frames[1].position += midDistance;
        }

        foreach(Sensor_Module mod in robot_ForSerialization.sensorModules)
        {
            mod.position += midDistance;
        }

        prevPos = new_pos;

        return midDistance;
    }

    void UpdateUIText()
    {
        left_Position_X.text = right_Position_X.text = l_pos.x.ToString();
        left_Position_Y.text = right_Position_Y.text = l_pos.y.ToString();
        left_Position_Z.text = right_Position_Z.text = l_pos.z.ToString();

        Ism_pos_x.text =sm_pos.x.ToString();
        Ism_pos_y.text =sm_pos.y.ToString();
        Ism_pos_z.text =sm_pos.z.ToString();
    }


    /*-----------------------------------------------------Settings-------------------------------------------------------*/
    /*---------------------------------------------------Save Config:-----------------------------------------------------*/
    public void SaveConfig()
    {

    }

    public void LoadConfig()
    {
    }
    /*---------------------------------------------------Panel toggles----------------------------------------------------*/
    public GameObject mainPanel;
    public void ToggleDesigner(bool toggled)
    {
        if (toggled)
            mainPanel.gameObject.SetActive(true);
        else
            mainPanel.gameObject.SetActive(false);
    }
    public GameObject optimizationPanel;
    public void ToggleOptiPanel(bool toggled)
    {
        if (toggled)
            optimizationPanel.gameObject.SetActive(true);
        else
            optimizationPanel.gameObject.SetActive(false);
    }
    public void ToggleOptimization(bool toggled)
    {
        if (toggled)
            Robot_Optimization.activated = true;
        else
            Robot_Optimization.activated = false;
    }
       

    /*-------------------------------------------------Helper Functions:--------------------------------------------------*/
    /*--------------------------------------------------Defining frames---------------------------------------------------*/
    Frame DefineFrame(string shape, AgX_Interface.Vector3 pos, double scale, AgX_Interface.Quaternion rot, double mass, string materialName)
    {
        return new Frame()//test create new object
        {
            guid = Guid.NewGuid(),
            shape = shape,
            position = pos,
            scale = scale,
            quatRotation = rot,
            //rotation = rot,
            mass = mass,
            isStatic = false,
            materialName = materialName
        };
    }
    Simulation_Core.Joint DefineJoint(Guid f1_guid, Guid f2_guid, string type, double l_rangeLimit, double r_rangeLimit, double max_vel)
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
    Scene DefineScene(byte[] bytes, AgX_Interface.Vector3 pos, string materialName, double height)
    {
        return new Scene()
        {
            guid = Guid.NewGuid(),
            height_Image = Convert.ToBase64String(bytes),
            position = pos,
            materialName = materialName,
            height = height
        };
    }
    Sensor_Module DefineSensoryModule(int leftModNr, int rightModNr, AgX_Interface.Vector3 pos, AgX_Interface.Vector3 size, double mass, AgX_Interface.Quaternion rot, string materialName )
    {
        return new Sensor_Module()
        {
            guid = Guid.NewGuid(),
            leftMod_Nr = leftModNr,
            rightMod_Nr = rightModNr, //if left is -1, it is the first module. if right is -1, it is the last module.
            position = pos,
            size = size,
            materialName = materialName,
            mass = mass,
            quatRotation = rot
        };
    }

    /*-------------------------------------------------Vector type conversion:--------------------------------------------------*/
    AgX_Interface.Vector3[] AgxHelper(UnityEngine.Vector3[] vec)//THESE?? wrong?
    {
        var vectors = new AgX_Interface.Vector3[vec.Length];

        for (int i = 0; i < vec.Length; i++)
        {
            vectors[i].x = vec[i].x;
            vectors[i].y = vec[i].y;
            vectors[i].z = vec[i].z;
        }
        return vectors;
    }
    List<UnityEngine.Vector3> AgxHelper(List<AgX_Interface.Vector3> vec)
    {
        var vectors = new List<UnityEngine.Vector3>();

        for (int i = 0; i < vec.Count; i++)
        {
            vectors.Add(new UnityEngine.Vector3((float)vec[i].x, (float)vec[i].y, (float)vec[i].z));
        }
        return vectors;
    }
    AgX_Interface.Vector2[] AgxHelper(UnityEngine.Vector2[] vec)
    {
        var vectors = new AgX_Interface.Vector2[vec.Length];

        for (int i = 0; i < vec.Length; i++)
        {
            vectors[i].x = vec[i].x;
            vectors[i].y = vec[i].y;
        }
        return vectors;
    }
    UnityEngine.Vector2[] AgxHelper(AgX_Interface.Vector2[] vec)
    {
        var vectors = new UnityEngine.Vector2[vec.Length];

        for (int i = 0; i < vec.Length; i++)
        {
            vectors[i].x = (float)vec[i].x;
            vectors[i].y = (float)vec[i].y;
        }
        return vectors;
    }
}
