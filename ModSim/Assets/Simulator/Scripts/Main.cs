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
    List<SceneObject> sceneObjects = new List<SceneObject>();
    //Robot OriginalRobot = new Robot();
    void Main_Initialization()
    {
        dir = Application.streamingAssetsPath;//Get the path of the streaming assets
        if (!simulation_Started)
        {
            Clear_Vis();
            Reset_Opti();
            Agx_Simulation.Start(dt);//Starts the sim.

            simulation_Started = true;
            CancelInvoke();
            Dynamics.nextAction = "Idle";


            //Load:
            scenario = Deserialize<Scenario>(Application.streamingAssetsPath + "/XML/Scenario.xml");

            SetContactFriction();

            // Loading the directories for the object files 
            Load_FrameDirectories(scenario.robot);

            robot = Load_Robot(scenario.robot);

            sceneObjects = LoadSceneObjects(scenario.sceneObjects);

            scene = new Scene(); Load_Scene(scenario.scene);

            //SetMovementVariables();


            Visualization.enabled = true;

            if (Visualization.enabled)
            {
                Load_Vis();
                Update_Vis(robot);
            }
        }
        
    }
    string terrainTexture = "dirt";
    public void TerrainTexture(string tex)
    {
        terrainTexture = tex;
    }

    /*-----------------------------------------------Optimization variables-----------------------------------------------*/
    public void SetMovementVariables(double[] forward_vars)
    {
        Robot_Optimization.originalGenome = forward_vars;
        Dynamics.f_movementVars = forward_vars;
        //add for turn, rotation, sidewinding, etc. vars.
    }
    public void SetOptimizationTarget(AgX_Interface.Vector3 pos)
    {
        Robot_Optimization.targetPosition = pos;
    }
    public void SetAxisWeighting(AgX_Interface.Vector3 axis)
    {
        Robot_Optimization.AxisWeight = axis;
    }
    public void SetOptimizationLimits(double[] limits)
    {
        var upper = new double[7];
        var lower = new double[7];

        for (int i = 0; i < 14; i++)
        {
            if (i < 7)
                upper[i] = limits[i];
            else
                lower[i-7] = limits[i];
        }

        Robot_Optimization.UpperLimit = upper;
        Robot_Optimization.LowerLimit = lower;
    }
    public void SetOptiToggle(bool[] toggled)
    {
        Robot_Optimization.toggledForOptimization = toggled;
    }


    public void StartSim()
    {
        if (!simulation_Running)
        {
            simulation_Running = true;
            //Start either the optimization or the dynamics
            if (Robot_Optimization.activated)
            {
                CreateOptimizations();
            }
            else
                InvokeRepeating("Update_Sim", 0.01f, dt);
        }
    }
    public void ResetAll()
    {
        simulation_Running = false;
        simulation_Started = false;
    }
    AgX_Interface.Vector3 RobotStartPosition = new AgX_Interface.Vector3();
    public void ChangeInitPos(AgX_Interface.Vector3 pos)
    {
        if (!simulation_Running)
        {
            AgX_Assembly.SetPosition(pos);
            robot.Update();
            Update_Vis(robot);
            RobotStartPosition = pos;
        }
    }
    AgX_Interface.Quaternion RobotStartRotation = new AgX_Interface.Quaternion();
    public void ChangeInitRot(AgX_Interface.Quaternion rot)
    {
        if (!simulation_Running)
        {
            AgX_Assembly.SetRotation(rot);
            robot.Update();
            Update_Vis(robot);
            RobotStartRotation = rot;
        }
    }

    void ChangeDeltaTime(float dt)
    {
        this.dt = dt;
        if (simulation_Running && !Robot_Optimization.activated)
        {
            CancelInvoke("Update_Sim");
            InvokeRepeating("Update_Sim", 0.01f, dt);
        }
    }

    void Stop()
    {
        Visualization.enabled = false;
        simulation_Running = false;
        
        Agx_Simulation.Stop();
        simulation_Started = false;

        Clear_Vis();
    }
    public void CancelRepeats()
    {
        CancelInvoke();
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
        Robot_Optimization.currentGeneration = 0;
    }

    void SetContactFriction()
    {
        //Foreach in scenario.contactmats:
        if(scenario.contactFrictions.Count == 0)
            Agx_Simulation.AddContactMaterial("Plastic", "Rock", 0.4f, 0.3f, 3.654E9);//Standard plastic and rock
        foreach (ContactFriction cf in scenario.contactFrictions)
            Agx_Simulation.AddContactMaterial(cf.material1,cf.material2,cf.restitution,cf.friction, cf.youngsModulus);
    }


    public static T Deserialize<T>(string path)
    {
        string fileName = path;// Application.streamingAssetsPath + "/XML/Scenario.xml";
        XmlSerializer serializer = new XmlSerializer(typeof(T));
        StreamReader reader = new StreamReader(fileName);
        T deserialized = (T)serializer.Deserialize(reader.BaseStream);
        reader.Close();
        return deserialized;
    }
    public static void Serialize(object item,string path)
    {
        string fileName = path;
        XmlSerializer serializer = new XmlSerializer(item.GetType());
        StreamWriter writer = new StreamWriter(fileName);
        serializer.Serialize(writer.BaseStream, item);
        writer.Close();
    }
    public void SaveToXml(string path)
    {
        if (simulation_Started)
            Serialize(scenario, path);
        else
            Debug.Log("Robot must be finalized!");
    }
    public GameObject finalizeButton;
    public void LoadFromXml(string path)
    {
        //get robot
        //design finished
        //deserialize
        if (!simulation_Started)
        {
            dir = Application.streamingAssetsPath;//Get the path of the streaming assets
            Clear_Vis();
            Reset_Opti();
            Agx_Simulation.Start(dt);//Starts the sim.

            simulation_Started = true;// true;
            simulation_Running = false;
            try { scenario = Deserialize<Scenario>(path); }
            catch (FileNotFoundException e)
            {
                Debug.Log("Scenario not found! " + e);
                AgX_Interface.AgX_Assembly.RemoveFromSim();
                AgX_Interface.Agx_Simulation.Stop();
                UnityEngine.SceneManagement.SceneManager.LoadScene(0);
            }


            SetContactFriction();//if custom contact points: move to MainInitialization().
            CancelInvoke();
            Dynamics.nextAction = "Idle";

            //scenario = Deserialize<Scenario>(Application.streamingAssetsPath + "/XML/Scenario.xml");

            

            robot = Load_Robot(scenario.robot);

            /* Loading the directories for the object files */
            Load_FrameDirectories(robot);

            sceneObjects = LoadSceneObjects(scenario.sceneObjects);

            //possibly remove this
            scene = new Scene(); Load_Scene(scenario.scene);

            Visualization.enabled = true;

            if (Visualization.enabled)
            {
                Load_Vis();
                Update_Vis(robot);
            }
            if (finalizeButton != null)
                finalizeButton.SetActive(false);
        }
        else
            Debug.Log("Must load before simulation is started");//UnityEditor.EditorUtility.DisplayDialog("Load/Save error!", "Must load before simulation is started", "Ok");


    }


    Robot robot;//Global for pos/rot update

    void Load_FrameDirectories(Robot robot)
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

        //SET MESH 
        foreach (Module mod in robot.modules)
        {
            mod.frames[0].SetMesh(AgxHelper(leftMesh.vertices),AgxHelper(leftMesh.uv),leftMesh.triangles); mod.frames[1].SetMesh(AgxHelper(rightMesh.vertices),AgxHelper(rightMesh.uv),rightMesh.triangles);
            
        }
        //Creates AgX objects and joint connections:
        robot.Initialize();//Initialize frames (creates AgX obj), initializes modules (connecting frames with joint), Locks modules together

        return robot;
        
    }

    List<SceneObject> LoadSceneObjects(List<SceneObject> s_obj)
    {
        foreach(SceneObject s in s_obj)
        {
            s.Initialize();
        }

        return s_obj;

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
        Load_SensorModuleVis(robot);
        Load_SceneObjectsVis();

        //Scene:
        scene_vis = new Scene_Vis(scene.guid, AgxHelper(scene.vertices), scene.triangles, AgxHelper(scene.uvs), AgxHelper(scene.position), Resources.Load(terrainTexture) as Texture);
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
    void Load_SensorModuleVis(Robot robot)
    {
        foreach (SensorModule mod in robot.sensorModules)
        {
            sensorModuleVis.Add(new SensorModule_Vis(mod.guid, AgxHelper(mod.position), AgxHelper(mod.size)));
            if (mod.forceSensor != null)
                forceSensorVis.Add(new ForceSensor_Vis(mod.forceSensor.guid, AgxHelper(mod.forceSensor.position), AgxHelper(mod.forceSensor.size)));
        }

    }

    void Load_SceneObjectsVis()
    {
        foreach(SceneObject o in sceneObjects)
        {
            sceneObjectVis.Add(new SceneObject_Vis(o.guid,AgxHelper(o.position),AgxHelper(o.size), o.shape));
        }
    }

    void Clear_Vis()
    {
        foreach (Frame_Vis vis in frameVis)
            vis.Remove();
        frameVis.Clear();

        foreach (SensorModule_Vis vis in sensorModuleVis)
            vis.Remove();
        sensorModuleVis.Clear();

        foreach (ForceSensor_Vis vis in forceSensorVis)
            vis.Remove();
        forceSensorVis.Clear();

        foreach (SceneObject_Vis vis in sceneObjectVis)
            vis.Remove();
        sceneObjectVis.Clear();

        if(scene_vis != null)
            scene_vis.Remove();
    }


    List<SensorModule_Vis> sensorModuleVis = new List<SensorModule_Vis>();
    List<ForceSensor_Vis> forceSensorVis = new List<ForceSensor_Vis>();
    List<SceneObject_Vis> sceneObjectVis = new List<SceneObject_Vis>();
    List<Frame_Vis> frameVis = new List<Frame_Vis>();
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
            //Update scene objects
            foreach (SceneObject so in sceneObjects)
                so.Update();

            /*
            //distancesensor
            string distances = "";
            foreach (DistanceSensor ds in robot.sensorModules[0].distanceSensors)
            {
                ds.CalculateDistance(sceneObjects);
                distances += ds.GetSensorDistance().ToString()+",";Debug.Log("working");
            }

            if(distances !="")
                Debug.Log(distances);
                */

            //Update visualization
            if (Visualization.enabled)
                Update_Vis(robot);

            //Debug.Log(robot.sensorModules[0].forceSensor.forceValue);

            simulationTime += Time.deltaTime;
        }

        //calculate distances 
        foreach (SensorModule mod in robot.sensorModules)
            foreach (DistanceSensor ds in mod.distanceSensors)
                ds.CalculateDistance(sceneObjects);

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
            case 6: Debug.Log("Read/Write error(folder not existing)"); break;
            case 7: Debug.Log("No Filename"); break;
            default:Debug.Log("Unspecified Error");break;
        }
    }

    //List<Robot> robots;
    int Opti_Iterator = 0;
    double[] genom = new double[7];
    void OptimizationUpdate_Sim()
    {
        if (simulation_Running)//Check if simulation is paused
        {
            //OPTIMIZATION:
            if (Robot_Optimization.Update(robot, simulationTime, Opti_Iterator, Robot_Optimization.IterTime))
            {
                simulationTime = 0;

                //deletes and recreates robot
                ResetRobot();
                
                //resets robot position:(gotta create AgX assembly first)
                //robot.setPos(oldpos?);

                Opti_Iterator++;
                if(!Robot_Optimization.quickOpti)
                Debug.Log("Generation: " + Robot_Optimization.currentGeneration + "| Entity: "+ Opti_Iterator);
                

                if (Opti_Iterator >= Robot_Optimization.population)//If all iterations have ran
                {
                    //start new iterations with new population
                    Robot_Optimization.UpdatePopulation(robot);//Update the populations

                    ResetRobot();
                    Opti_Iterator = 0;

                    genom = Robot_Optimization.currentBestGenome;
                    currDynamics.text = genom[0].ToString("0.###") + "," + genom[1].ToString("0.###") + "," + genom[2].ToString("0.###") + "," + genom[3].ToString("0.###")
                        + "," + genom[4].ToString("0.###") + "," + genom[5].ToString("0.###") + "," + genom[6].ToString("0.###");
                }
            }

            //VISUALIZATION:
            if (Visualization.enabled)
            {
                Update_Vis(robot);
            }



            //this should be the Physics time
            simulationTime += Robot_Optimization.deltaTime;//timestep = dt = each step time. 
        }
    }
    public UnityEngine.UI.InputField currDynamics;
    void ResetRobot()//Cannot reset robot while sim is running. 
    {

        /* 1.attempt: */
        
        robot.RemovePhysicsObjects();
        robot = new Robot();
        robot = Load_Robot(Deserialize<Scenario>(Application.streamingAssetsPath + "/XML/Scenario.xml").robot);//new robot
        

        /*
        robot.RemovePhysicsObjects();
        robot = OriginalRobot;//new robot
        robot = Load_Robot(robot);
        */

        /* 2. attempt: *"
        AgX_Assembly.RemoveFromSim();

        AgX_Assembly.SetPosition(RobotStartPosition);
        AgX_Assembly.SetRotation(RobotStartRotation);
        foreach(Module mod in robot.modules)
        {
            mod.joint.Stabilize_Angle();
        }

        AgX_Assembly.AddToSim();


        /* 3. attempt: */
        /*
        Agx_Simulation.Stop();
        Agx_Simulation.Start(dt);
        var scen = Deserialize<Scenario>();

        Load_Scene(scen.scene);
        robot = Load_Robot(scen.robot);
        */

        

    }

    void CreateOptimizations()
    {
        //Loadrobot for each robot
        //Loadvis for each robot
        Robot_Optimization.Load(robot);

        Robot_Optimization.deltaTime = dt;

        //Robot_Optimization.IterTime = 10;
        //Robot_Optimization.timeStep = 0.01f ;// dt;

        InvokeRepeating("OptimizationUpdate_Sim", 0.01f, Robot_Optimization.deltaTime);

    }

    void QuickOptimization(int goal_generation)
    {
        if (simulation_Running && Robot_Optimization.activated)
        {
            if (!Robot_Optimization.started)
                Robot_Optimization.Load(robot);

            CancelInvoke();
            Visualization.enabled = false;

            simulation_Running = true;

            Robot_Optimization.quickOpti = true;
            //Optimize fast:
            while(Robot_Optimization.currentGeneration < goal_generation)
            {
                OptimizationUpdate_Sim();
            }
            Debug.Log("Reached Generation: " + Robot_Optimization.currentGeneration);

            //simulation_Running = false;

            Robot_Optimization.quickOpti = false;

            //reenable normal optimization:
            Visualization.enabled = true;
            InvokeRepeating("OptimizationUpdate_Sim", 0.01f, Robot_Optimization.deltaTime);
        }
    }

    void Update_Vis(Robot robot)
    {
        
        foreach (Module module in robot.modules)
        {
            foreach (Frame frame in module.frames)
            {
                //Retrieves Frameobject with GUID, and updates position,size,rotation:
                try { frameVis.Find(x => x.guid == frame.guid).Update(AgxHelper(frame.position), AgxHelper(frame.quatRotation),module.axis); } catch (NullReferenceException e) { Debug.Log("Could not find frame with Guid. " + e); }
            }
        }
        foreach(SensorModule mod in robot.sensorModules)
        {
            try { sensorModuleVis.Find(x => x.guid == mod.guid).Update(AgxHelper(mod.position), AgxHelper(mod.quatRotation)); } catch(NullReferenceException e) { Debug.Log("Could not find Sensor Module with Guid. " + e); }

            if (mod.forceSensor != null)
                try { forceSensorVis.Find(x => x.guid == mod.forceSensor.guid).Update(AgxHelper(mod.forceSensor.position), AgxHelper(mod.forceSensor.rotation)); } catch (NullReferenceException e) { Debug.Log("Could not find Force Sensor with Guid." + e); }
        }

        foreach(SceneObject so in sceneObjects)
        {
            try { sceneObjectVis.Find(x => x.guid == so.guid).Update(AgxHelper(so.position), AgxHelper(so.quatRotation)); } catch (NullReferenceException e) { Debug.Log("Could not find Scene Object with Guid. " + e); }
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
        if(Input.GetButtonUp("Custom"))
        {
            Dynamics.SetMovement("Custom", 0, 0);
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
