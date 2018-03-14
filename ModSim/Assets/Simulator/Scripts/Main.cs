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
            mod.frames[0].setMesh(leftMesh.vertices, leftMesh.uv, leftMesh.triangles); mod.frames[1].setMesh(rightMesh.vertices, rightMesh.uv, rightMesh.triangles);

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


            frameVis.Add(new Frame_Vis(mod.frames[0].guid, l, mod.frames[0].position,mod.frames[0].scale));
            frameVis.Add(new Frame_Vis(mod.frames[1].guid, r, mod.frames[1].position,mod.frames[1].scale));

        }

        //Scene:
        Scene_Vis scene_vis = new Scene_Vis(scene.guid, scene.vertices, scene.triangles, scene.uvs, scene.position, Resources.Load("grass") as Texture);

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
                try { frameVis.Find(x => x.guid == frame.guid).Update(frame.position, frame.rotation,module.Axis); } catch (NullReferenceException e) { Debug.Log("Could not find frame with Guid." + e); }
            }

            //try { jointVis.Find(x => x.guid == module.joint.guid).Update(module.joint.Vis_ContactPoints()); } catch(NullReferenceException e) { Debug.Log("Could not find joint with Guid." + e ); }
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
    
}
