using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Linq;
using AgX_Interface;
using Simulation_Core;
using Unity_Visualization;
using System;
using System.Xml.Serialization;

public class Main : MonoBehaviour {

    public static bool simulation_Running = false;
    public agx.AutoInit agxInit;
    public agxSDK.Simulation mysim;
    public float dt = 1.0f / 100.0f;

    string dir = "";
    string upperFrame_directory = "/Robot/upper.obj";
    string bottomFrame_directory = "/Robot/bottom.obj";

    void Start()// Use this for initialization
    {
        Physics.autoSimulation = false; //Turn off Unity Physics
        //Debug.Log(new UnityEngine.Quaternion(0, -0.013707354664802551f, 0, 0.999906063079834f).eulerAngles);
        
        //Main_Initialization();
    }

    void Main_Initialization()
    {
        Agx_Simulation.Start(dt);//Starts the sim.
        dir = Application.streamingAssetsPath;//Get the path of the streaming assets

        

        //If I start with 3 modules. Then, each time user clicks "Add Module", it adds a new module to the simulation (sim will be started, but not timestep).

        //LOAD:
        Scenario scenario = Deserialize<Scenario>();

        /* Loading the directories for the object files */
        load_FrameDirectories(scenario.robot);
        Load_Robot(scenario.robot);
        Load_Scene(scenario.scene);

        simulation_Running = true;
        Visualization.enabled = true;

        if (Visualization.enabled)
            Load_Vis();

        SetContactPoints();//if custom contact points: move to MainInitialization().

        InvokeRepeating("Update_Sim", 0, dt);
        
    }

    void SetContactPoints()
    {
        Agx_Simulation.AddContactMaterial("Plastic","Rock",0.4f,0.3f, (float)3.654E9);
    }


    public static T Deserialize<T>()
    {
        string fileName = Application.streamingAssetsPath + "/XML/Scenario.xml";
        XmlSerializer serializer = new XmlSerializer(typeof(T));
        StreamReader reader = new StreamReader(fileName);
        T deserialized = (T)serializer.Deserialize(reader.BaseStream);
        reader.Close();
        return deserialized;
    }

    Robot robot;//Global for pos/rot update
    
    

    void load_FrameDirectories(Robot robot)
    {
        upperFrame_directory = "/Robot/" + robot.leftFrameDir;
        bottomFrame_directory = "/Robot/" + robot.rightFrameDir;
    }

    void Load_Robot(Robot robot)
    {
        //Initialize modules with joints and frames (+agx objects) : SHOULD BE IN SCENE DESIGNER, send triangles, verts and uvs!
        ObjImporter import = new ObjImporter();
        
        Mesh leftMesh = import.ImportFile(dir + upperFrame_directory);Bounds leftBound = leftMesh.bounds;
        Mesh rightMesh = import.ImportFile(dir + bottomFrame_directory);Bounds rightBound = rightMesh.bounds;

        //new z pos is start.z - meshLength*i. 
        foreach (Module mod in robot.modules)
        {
            mod.frames[0].setMesh(Sim_CoreHelper(leftMesh.vertices),Sim_CoreHelper(leftMesh.uv),leftMesh.triangles); mod.frames[1].setMesh(Sim_CoreHelper(rightMesh.vertices),Sim_CoreHelper(rightMesh.uv),rightMesh.triangles);

            /*foreach (Frame frame in mod.frames)
            {
                //frame.Initialize();
            }
            */
            //mod.Initialize(mod.frames[0], mod.frames[1]);//calls Create_Hinge
            
        }

        robot.Initialize();//Initialize frames (creates AgX obj), initializes modules (connecting frames with joint), Locks modules together

        this.robot = robot;
        
    }

    Scene scene;
    public void Load_Scene(Scene scene)
    {
        //Initialize scene:
        scene.Create();
        this.scene = scene;
    }


    void Load_Vis()
    {
        //Frames:
        foreach (Module mod in robot.modules)
        {
           /* Mesh l = new Mesh() { vertices = mod.frames[0].meshVertices, uv = mod.frames[0].meshUvs, triangles = mod.frames[0].meshTriangles };
            Mesh r = new Mesh() { vertices = mod.frames[1].meshVertices, uv = mod.frames[1].meshUvs, triangles = mod.frames[1].meshTriangles };*/
            ObjImporter import = new ObjImporter();
            Mesh l = import.ImportFile(dir + upperFrame_directory);//Should make variable: String upperDirectory = ...
            Mesh r = import.ImportFile(dir + bottomFrame_directory);

            frameVis.Add(new Frame_Vis(mod.frames[0].guid, l,Sim_CoreHelper(mod.frames[0].position),mod.frames[0].scale));
            frameVis.Add(new Frame_Vis(mod.frames[1].guid, r, Sim_CoreHelper(mod.frames[1].position),mod.frames[1].scale));

        }

        foreach(Sensory_Module mod in robot.sensorModules)
        {
            sensorVis.Add(new Sensor_Vis(mod.guid, Sim_CoreHelper(mod.position), Sim_CoreHelper(mod.size)));
        }

        //Scene:
        Scene_Vis scene_vis = new Scene_Vis(scene.guid, Sim_CoreHelper(scene.vertices), scene.triangles, Sim_CoreHelper(scene.uvs), Sim_CoreHelper(scene.position), Resources.Load("grass") as Texture);

    }


    List<Sensor_Vis> sensorVis = new List<Sensor_Vis>();
    List<Frame_Vis> frameVis = new List<Frame_Vis>();
    List<Joint_Vis> jointVis = new List<Joint_Vis>();
    float simulationTime = 0;
    void Update_Sim()
    {
        if (simulation_Running)//Check if simulation is paused
        {
            Agx_Simulation.StepForward();
            //Check if a button has been pressed
            //CheckInputs();

            if (simulationTime >= 2)//Wait for robot to settle on terrain
                if (!Dynamics.Control(robot, simulationTime))//Movement
                    Debug.Log("wrong command");

            robot.Update();

            if (Visualization.enabled)
                Update_Vis();

            simulationTime += Time.deltaTime;
        }
        /*foreach (Module mod in robot.modules)
            Debug.Log("x: " + mod.frames[0].rotation.x + ", y: " + mod.frames[0].rotation.y + ", z: " + mod.frames[0].rotation.z);*/
        //Else: 
        //Start the canvas overlay to modify and create new modules
    }

    GameObject go;
    void Update_Vis()
    {
        
        foreach (Module module in robot.modules)
        {
            foreach (Frame frame in module.frames)
            {
                //Retrieves Frameobject with GUID, and updates position,size,rotation:
                try { frameVis.Find(x => x.guid == frame.guid).Update(Sim_CoreHelper(frame.position), Sim_CoreHelper(frame.quatRotation),module.Axis); } catch (NullReferenceException e) { Debug.Log("Could not find frame with Guid." + e); }
            }

            //try { jointVis.Find(x => x.guid == module.joint.guid).Update(module.joint.Vis_ContactPoints()); } catch(NullReferenceException e) { Debug.Log("Could not find joint with Guid." + e ); }
        }
        foreach(Sensory_Module mod in robot.sensorModules)
        {
            try { sensorVis.Find(x => x.guid == mod.guid).Update(Sim_CoreHelper(mod.position), Sim_CoreHelper(mod.quatRotation)); } catch(NullReferenceException e) { Debug.Log("Could not find Sensor Module with Guid." + e); }
        }

    }


    void Update()
    {
        CheckInputs();
    }

    /* Movement commands for the robot: */
    void CheckInputs()
    {
        if(Input.GetButtonUp("Turn"))
        {
            Dynamics.SetMovement("Turn",0,Math.Sign(Input.GetAxis("Turn")));
        }
        if(Input.GetButtonUp("Forward"))
        {
            Dynamics.SetMovement("Forward", Math.Sign(Input.GetAxis("Forward")), 0);
        }
        if(Input.GetButtonUp("Reset"))
        {
            Dynamics.SetMovement("Reset", 0,0);
        }
        if(Input.GetButtonUp("Idle"))
        {
            Dynamics.SetMovement("Idle", 0, 0);
        }
        if(Input.GetButtonUp("Speed"))
        {
           Dynamics.ChangeSpeed((float)Math.Sign(Input.GetAxis("Speed")));
        }
    }


    void OnApplicationQuit()///When Unity closes, shutdown AgX.
    {
        Agx_Simulation.Stop();
        Debug.Log("Application ending after " + Time.time + " seconds");

    }

    //.dll Helper Functions
    AgX_Interface.Vector3 Sim_CoreHelper(UnityEngine.Vector3 vec)
    {
        AgX_Interface.Vector3 vector;
        vector.x = vec.x;
        vector.y = vec.y;
        vector.z = vec.z;
        return vector;
    }
    AgX_Interface.Vector3[] Sim_CoreHelper(UnityEngine.Vector3[] vec)//THESE?? wrong?
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
    List<UnityEngine.Vector3> Sim_CoreHelper(List<AgX_Interface.Vector3> vec)
    {
        var vectors = new List<UnityEngine.Vector3>();

        for (int i = 0; i < vec.Count; i++)
        {
            vectors.Add(new UnityEngine.Vector3((float)vec[i].x, (float)vec[i].y, (float)vec[i].z));
        }
        return vectors;
    }
    AgX_Interface.Vector2[] Sim_CoreHelper(UnityEngine.Vector2[] vec)
    {
        var vectors = new AgX_Interface.Vector2[vec.Length];

        for (int i = 0; i < vec.Length; i++)
        {
            vectors[i].x = vec[i].x;
            vectors[i].y = vec[i].y;
        }
        return vectors;
    }
    UnityEngine.Vector2[] Sim_CoreHelper(AgX_Interface.Vector2[] vec)
    {
        var vectors = new UnityEngine.Vector2[vec.Length];

        for (int i = 0; i < vec.Length; i++)
        {
            vectors[i].x = (float)vec[i].x;
            vectors[i].y = (float)vec[i].y;
        }
        return vectors;
    }
    UnityEngine.Vector3 Sim_CoreHelper(AgX_Interface.Vector3 vec)
    {
        var vector = new UnityEngine.Vector3();
        vector.x = (float)vec.x;
        vector.y = (float)vec.y;
        vector.z = (float)vec.z;

        return vector;
    }
    UnityEngine.Quaternion Sim_CoreHelper(AgX_Interface.Quaternion quat)
    {
        UnityEngine.Quaternion Uq = new UnityEngine.Quaternion();
        Uq.x = (float)quat.x;
        Uq.y = (float)quat.y;
        Uq.z = (float)quat.z;
        Uq.w = (float)quat.w;
        return Uq;
    }

}
