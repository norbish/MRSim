/* Main class for Simulation control (Main)
 * Torstein Sundnes Lenerand
 * NTNU Ålesund
 */

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

    public static bool simulation_Running = false;bool simulation_Started = false;
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
    Scenario scenario = new Scenario();
    void Main_Initialization()
    {
        dir = Application.streamingAssetsPath;//Get the path of the streaming assets
        if (!simulation_Started)
        {
            Clear_Vis();
            Reset_Opti();
            Agx_Simulation.Start(dt);//Starts the sim.
            
            simulation_Started = true;

            SetContactPoints();//if custom contact points: move to MainInitialization().
            CancelInvoke();
            Dynamics.action = "Idle";
        }
        else
        {
            Clear_Vis();
            Reset_Opti();
            Agx_Simulation.Stop();
            Agx_Simulation.Start(dt);
            simulation_Started = true;
            SetContactPoints();
            CancelInvoke();
            Dynamics.action = "Idle";
        }
        //If I start with 3 modules. Then, each time user clicks "Add Module", it adds a new module to the simulation (sim will be started, but not timestep).

        //LOAD:
        scenario = Deserialize<Scenario>();

        /* Loading the directories for the object files */
        load_FrameDirectories(scenario.robot);
        robot = Load_Robot(scenario.robot);

        scene = new Scene(); Load_Scene(scenario.scene);


        simulation_Running = true;
        Visualization.enabled = true;

        if (Visualization.enabled)
            Load_Vis();


        //InvokeRepeating("Update_Sim", 0, dt);
        CreateOptimizations();
        
    }

    void Stop()
    {
        Visualization.enabled = false;
        simulation_Running = false;
        
        Agx_Simulation.Stop();
        simulation_Started = false;

        Clear_Vis();
    }
    void Pause()
    {
        if (simulation_Running)
            simulation_Running = false;
        else
            simulation_Running = true;
    }

    void Reset_Opti()
    {
        Robot_Optimization.Reset();
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

    Robot Load_Robot(Robot robot)
    {
        //Initialize modules with joints and frames (+agx objects) : SHOULD BE IN SCENE DESIGNER, send triangles, verts and uvs!
        ObjImporter import = new ObjImporter();
        
        Mesh leftMesh = import.ImportFile(dir + upperFrame_directory);Bounds leftBound = leftMesh.bounds;
        Mesh rightMesh = import.ImportFile(dir + bottomFrame_directory);Bounds rightBound = rightMesh.bounds;

        //new z pos is start.z - meshLength*i. 
        foreach (Module mod in robot.modules)
        {
            mod.frames[0].setMesh(AgxHelper(leftMesh.vertices),AgxHelper(leftMesh.uv),leftMesh.triangles); mod.frames[1].setMesh(AgxHelper(rightMesh.vertices),AgxHelper(rightMesh.uv),rightMesh.triangles);
            
        }

        robot.Initialize();//Initialize frames (creates AgX obj), initializes modules (connecting frames with joint), Locks modules together

        return robot;
        
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
        Load_RobotVis(robot);
        Load_SensorVis(robot);
        

        //Scene:
        scene_vis = new Scene_Vis(scene.guid, AgxHelper(scene.vertices), scene.triangles, AgxHelper(scene.uvs), AgxHelper(scene.position), Resources.Load("grass") as Texture);
    }

    void Load_RobotVis(Robot robot)
    {
        foreach (Module mod in robot.modules)
        {
           /* Mesh l = new Mesh() { vertices = mod.frames[0].meshVertices, uv = mod.frames[0].meshUvs, triangles = mod.frames[0].meshTriangles };
            Mesh r = new Mesh() { vertices = mod.frames[1].meshVertices, uv = mod.frames[1].meshUvs, triangles = mod.frames[1].meshTriangles };*/
            ObjImporter import = new ObjImporter();
            Mesh l = import.ImportFile(dir + upperFrame_directory);//Should make variable: String upperDirectory = ...
            Mesh r = import.ImportFile(dir + bottomFrame_directory);

            frameVis.Add(new Frame_Vis(mod.frames[0].guid, l,AgxHelper(mod.frames[0].position),mod.frames[0].scale));
            frameVis.Add(new Frame_Vis(mod.frames[1].guid, r, AgxHelper(mod.frames[1].position),mod.frames[1].scale));
        }
    }
    void Load_SensorVis(Robot robot)
    {
        foreach (Sensor_Module mod in robot.sensorModules)
        {
            sensorVis.Add(new Sensor_Vis(mod.guid, AgxHelper(mod.position), AgxHelper(mod.size)));
        }

    }

    void Clear_Vis()
    {
        foreach (Frame_Vis vis in frameVis)
        {
            vis.Remove();
        }
        frameVis.Clear();

        foreach (Sensor_Vis vis in sensorVis)
        {
            vis.Remove();
        }
        sensorVis.Clear();

        if(scene_vis != null)
            scene_vis.Remove();
    }


    List<Sensor_Vis> sensorVis = new List<Sensor_Vis>();
    List<Frame_Vis> frameVis = new List<Frame_Vis>();
    List<Joint_Vis> jointVis = new List<Joint_Vis>();
    Scene_Vis scene_vis;

    double[] dynamicVariables = new double[7] { 2 * (Math.PI / 9.0f), 0, Math.PI * 2.0f / 3.0f, 0, 4.0f, 0, 0 };

    float simulationTime = 0;
    void Update_Sim()
    {
        if (simulation_Running)//Check if simulation is paused
        {
            Agx_Simulation.StepForward();
            //Check if a button has been pressed
            //CheckInputs();

            if (simulationTime >= 2)//Wait for robot to settle on terrain
                if (!Dynamics.Control(robot, simulationTime,dynamicVariables))//Movement
                    Debug.Log("wrong command");

            robot.Update();

            if (Visualization.enabled)
                Update_Vis(robot);

            simulationTime += Time.deltaTime;
        }

        //IF Analytics checked, saveData.(if count = 10, count = 0 and saveData?)
        int result = Analytics_Visualization.SaveData(robot, Time.time);
        switch(result)
        {
            case 0: break;
            case 1: break;
            case 2: break;
            case 3: break;
            case 4: break;
            case 5: break;
            case 6: Debug.Log("Read/Write error"); break;
            case 7: Debug.Log("No Filename"); break;
            default:Debug.Log("Unspecified Error");break;
        }
    }

    //List<Robot> robots;
    double Opti_IterationTime = 15;
    int Opti_Iterator = 0;
    void OptimizationUpdate_Sim()
    {
        if (simulation_Running)//Check if simulation is paused
        {
            //update dynamics/angle of robot, until 
            if (Robot_Optimization.Update(robot, simulationTime, Opti_Iterator, Opti_IterationTime))
            {
                simulationTime = 0;

                robot.RemovePhysicsObjects();
                robot = new Robot();
                robot = Load_Robot(Deserialize<Scenario>().robot);//new robot
                //RESET VIS TOO

                Opti_Iterator++;
                Debug.Log("Generation: " + Robot_Optimization.currentGeneration + "| Entity: "+ Opti_Iterator);

                if (Opti_Iterator >= Robot_Optimization.population)//If all iterations have ran
                {
                    //start new iterations with new population
                    Robot_Optimization.UpdatePopulation(robot);//Update the populations
                    
                    robot.RemovePhysicsObjects();
                    robot = new Robot();
                    robot = Load_Robot(Deserialize<Scenario>().robot);//new robot
                    Opti_Iterator = 0;
                }
            }

            //this will auto be for all:
            if (Visualization.enabled)
            {
                Update_Vis(robot);
            }

            simulationTime += Time.deltaTime;
        }
    }

    void CreateOptimizations()
    {
        //Loadrobot for each robot
        //Loadvis for each robot
        Robot_Optimization.Load(robot);

        Robot_Optimization.runspeed = 0.001f ;// dt;

        InvokeRepeating("OptimizationUpdate_Sim", 0, Robot_Optimization.runspeed);

    }

    GameObject go;
    void Update_Vis(Robot robot)
    {
        
        foreach (Module module in robot.modules)
        {
            foreach (Frame frame in module.frames)
            {
                //Retrieves Frameobject with GUID, and updates position,size,rotation:
                try { frameVis.Find(x => x.guid == frame.guid).Update(AgxHelper(frame.position), AgxHelper(frame.quatRotation),module.Axis); } catch (NullReferenceException e) { Debug.Log("Could not find frame with Guid." + e); }
            }

            //try { jointVis.Find(x => x.guid == module.joint.guid).Update(module.joint.Vis_ContactPoints()); } catch(NullReferenceException e) { Debug.Log("Could not find joint with Guid." + e ); }
        }
        foreach(Sensor_Module mod in robot.sensorModules)
        {
            try { sensorVis.Find(x => x.guid == mod.guid).Update(AgxHelper(mod.position), AgxHelper(mod.quatRotation)); } catch(NullReferenceException e) { Debug.Log("Could not find Sensor Module with Guid." + e); }
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
    AgX_Interface.Vector3 AgxHelper(UnityEngine.Vector3 vec)
    {
        AgX_Interface.Vector3 vector;
        vector.x = vec.x;
        vector.y = vec.y;
        vector.z = vec.z;
        return vector;
    }
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
    UnityEngine.Vector3 AgxHelper(AgX_Interface.Vector3 vec)
    {
        var vector = new UnityEngine.Vector3();
        vector.x = (float)vec.x;
        vector.y = (float)vec.y;
        vector.z = (float)vec.z;

        return vector;
    }
    UnityEngine.Quaternion AgxHelper(AgX_Interface.Quaternion quat)
    {
        UnityEngine.Quaternion Uq = new UnityEngine.Quaternion();
        Uq.x = (float)quat.x;
        Uq.y = (float)quat.y;
        Uq.z = (float)quat.z;
        Uq.w = (float)quat.w;
        return Uq;
    }

}
