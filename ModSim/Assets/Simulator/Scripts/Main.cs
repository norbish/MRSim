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

        
        Main_Initialization();
    }

    void Main_Initialization()
    {
        Agx_Simulation.Start(dt);//Starts the sim.
        dir = Application.streamingAssetsPath;//Get the path of the streaming assets

        SetContactPoints();//if custom contact points: move to MainInitialization().

        //If I start with 3 modules. Then, each time user clicks "Add Module", it adds a new module to the simulation (sim will be started, but not timestep).
        Serialization();//Move this to Scene designer

        //LOAD:
        Scenario scenario = Deserialize<Scenario>();
        Load_Robot(scenario.robot);
        Load_Scene(scenario.scene);

        simulation_Running = true;
        Visualization.enabled = true;

        if (Visualization.enabled)
            Load_Vis();


        InvokeRepeating("Update_Sim", 0, dt);
        
    }

    void SetContactPoints()
    {
        Agx_Simulation.AddContactMaterial("Plastic","Rock",0.4f,0.3f, (float)3.654E9);
    }


    Frame DefineFrame(string shape, UnityEngine.Vector3 pos, float scale, UnityEngine.Vector3 rot, float mass, string materialName)
    {
        return new Frame()//test create new object
        {
            guid = Guid.NewGuid(),
            shape = shape,
            position = pos,
            scale = scale,
            rotation =  rot,
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
    Scene DefineScene(byte[]bytes, UnityEngine.Vector3 pos, string materialName, float height)
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

    void Serialization()//Make into static class?
    {
        Robot robot_serialize = new Robot();

        List<Frame> frames = new List<Frame>();
        List<Simulation_Core.Joint> joints = new List<Simulation_Core.Joint>();

        Vector3 start = new Vector3(15, 12, 40);

        //For finding the size of the modules:
        ObjImporter import = new ObjImporter();

        Mesh leftMesh = import.ImportFile(dir + upperFrame_directory); Bounds leftBound = leftMesh.bounds;
        Mesh rightMesh = import.ImportFile(dir + bottomFrame_directory); Bounds rightBound = rightMesh.bounds;

        //Creating modules
        for (int i = 0; i < 10 ; i++)
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

            Module module = DefineModule(f1,j1,f2);

            //CREATES A LOCK BETWEEN MODULES when adding new module to robot:
            if(i > 0)
            {
                robot_serialize.Add_Module(module, new Simulation_Core.Joint());
            }else
                robot_serialize.Add_Module(module/*,lockJoint*/);  
    }

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

        //Sensors:
        foreach (Module mod in robot.modules)
        {
            if (mod.sensor != null)
            {
                sensorVis.Add(new Sensor_Vis(mod.sensor.guid, mod.sensor.position, mod.sensor.scale));
            }
        }

        //Scene:
        Scene_Vis scene_vis = new Scene_Vis(scene.guid, scene.vertices, scene.triangles, scene.uvs, scene.position, Resources.Load("grass") as Texture);

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

            foreach (Frame frame in mod.frames)
            {
                frame.Initialize();
            }

            mod.Initialize(mod.frames[0], mod.frames[1]);//calls Create_Hinge

            //Sensor vis:
            if (mod.sensor != null)
            {
                mod.Initialize_Sensors();
            }

        }
        robot.Initialize();//Locks modules together

        //Gotta know if the sensor is attached to pitch or yaw, so init it after the robot is initialized:
        this.robot = robot;
    }

    Scene scene;
    public void Load_Scene(Scene scene)
    {
        //Initialize scene:
        scene.Create();
        this.scene = scene;
    }

    List<Sensor_Vis> sensorVis = new List<Sensor_Vis>();
    List<Frame_Vis> frameVis = new List<Frame_Vis>();
    List<Joint_Vis> jointVis = new List<Joint_Vis>();
    


    void Update_Sim()
    {
        if (simulation_Running)//Check if simulation is paused
        {
            Agx_Simulation.StepForward();

            if (Time.fixedTime >= 2)//Wait for robot to settle on terrain
                if (!Dynamics.Control(robot, Time.fixedTime))//Movement
                    Debug.Log("wrong command");

            robot.Update();

            if (Visualization.enabled)
                Update_Vis();
        }
        //Else: 
        //Start the canvas overlay to modify and create new modules
    }

    void Update_Vis()
    {
        foreach (Module module in robot.modules)
        {
            foreach (Frame frame in module.frames)
            {
                //Retrieves Frameobject with GUID, and updates position,size,rotation:
                try { frameVis.Find(x => x.guid == frame.guid).Update(frame.position, frame.rotation,module.Axis); } catch (NullReferenceException e) { Debug.Log("Could not find frame with Guid." + e); }
            }

            if (module.sensor != null)
            {
                sensorVis.Find(x => x.guid == module.sensor.guid).Update(module.sensor.position);
            }
            //try { jointVis.Find(x => x.guid == module.joint.guid).Update(module.joint.Vis_ContactPoints()); } catch(NullReferenceException e) { Debug.Log("Could not find joint with Guid." + e ); }
        }
    }

    void OnApplicationQuit()///When Unity closes, shutdown AgX.
    {
        Agx_Simulation.Stop();
        Debug.Log("Application ending after " + Time.time + " seconds");

    }
    
}
