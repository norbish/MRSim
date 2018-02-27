using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using agxUtil;
using agxCollide;
using AgX_Interface;
using Simulation_Core;
using Unity_Visualization;
using System;
using agxIO;
using System.Xml.Serialization;

public class Main : MonoBehaviour {

    public agx.AutoInit agxInit;
    public agxSDK.Simulation mysim;
    public float dt = 1.0f / 100.0f;
    public int move = 0;

    void Start()// Use this for initialization
    {
        Physics.autoSimulation = false; //Turn off Unity Physics

        Agx_Simulation.Start(dt);//Starts the sim.

        Serialization();//Move this to Scene designer

        //LOAD:
        Scenario scenario = Deserialize<Scenario>();
        Load_Robot(scenario.robot);
        Load_Scene(scenario.scene);
        Set_Dynamics();

        InvokeRepeating("Update_AGX", 0, dt);
        
    }

    void Serialization()
    {
        Robot robot_serialize = new Robot();

        List<Frame> frames = new List<Frame>();
        List<Simulation_Core.Joint> joints = new List<Simulation_Core.Joint>();

        Vector3 frame1Pos = new Vector3(15, 15, 40);
        Vector3 frame2Pos = new Vector3(15, 15, 36);

    for(int i = 0; i < 5; i++)
    {

            var f1 = new Frame()//test create new object
            {
                guid = Guid.NewGuid(),
                shape = "Box",
                position = new Vector3(frame1Pos.x, frame1Pos.y, frame1Pos.z - 6 * i),// new Vector3(15, 15, 20 -6*i),//20 = start position
                size = Vector3.one,
                /*rotation = Vector3.zero,//*/rotation = i % 2 == 0 ? Vector3.zero : new Vector3(0,0,Mathf.PI/2),
                mass = 50,
                isStatic = false,
                materialName = "plastic",
                friction = 0.9f,
                restitution = 0.1f
            };

        var f2 = new Frame()//test create new object
        {
            guid = Guid.NewGuid(),
            shape = "Box",
            position = new Vector3(frame2Pos.x, frame2Pos.y, frame2Pos.z - 6 * i),// new Vector3(15, 15, 16-6*i),
            size = Vector3.one,
            /*rotation = Vector3.zero,//*/rotation = i % 2 == 0 ? Vector3.zero : new Vector3(0, 0, Mathf.PI/2),
            mass = 50,
            isStatic = false,
            materialName = "plastic",
            friction = 0.9f,
            restitution = 0.1f
        };

        var j1 = new Simulation_Core.Joint()
        {
            guid = Guid.NewGuid(),
            leftFrameGuid = f1.guid, rightFrameGuid = f2.guid,
            type = "Hinge",
        };

            var module = new Module();
            module.Create(f1, j1, f2);


            
            if(i > 0)
            {
                robot_serialize.Add_Module(module, new Simulation_Core.Joint());
            }else
                robot_serialize.Add_Module(module/*,lockJoint*/);
            
    }
    /*
        var f3 = new Frame()//test create new object
        {
            guid = Guid.NewGuid(),
            shape = "Box",
            position = new Vector3(15, 15, 14),
            size = Vector3.one,
            rotation = Vector3.zero,
            mass = 50,
            isStatic = false,
            materialName = "plastic",
            friction = 0.9f,
            restitution = 0.1f
        }; frames.Add(f3);

        var f4 = new Frame()//test create new object
        {
            guid = Guid.NewGuid(),
            shape = "Box",
            position = new Vector3(15, 15, 10),
            size = Vector3.one,
            rotation = Vector3.zero,
            mass = 50,
            isStatic = false,
            materialName = "plastic",
            friction = 0.9f,
            restitution = 0.1f
        }; frames.Add(f4);

        var j2 = new Simulation_Core.Joint()
        {
            guid = Guid.NewGuid(),
            leftFrameGuid = f3.guid,rightFrameGuid = f4.guid,
            type = "Hinge"
        };joints.Add(j2);

        var f5 = new Frame()//test create new object
        {
            guid = Guid.NewGuid(),
            shape = "Box",
            position = new Vector3(15, 15, 8),
            size = Vector3.one,
            rotation = Vector3.zero,
            mass = 50,
            isStatic = false,
            materialName = "plastic",
            friction = 0.9f,
            restitution = 0.1f
        }; frames.Add(f5);

        var f6 = new Frame()//test create new object
        {
            guid = Guid.NewGuid(),
            shape = "Box",
            position = new Vector3(15, 15, 4),
            size = Vector3.one,
            rotation = Vector3.zero,
            mass = 50,
            isStatic = false,
            materialName = "plastic",
            friction = 0.9f,
            restitution = 0.1f
        }; frames.Add(f6);

        var j3 = new Simulation_Core.Joint()
        {
            guid = Guid.NewGuid(),
            leftFrameGuid = f5.guid, rightFrameGuid = f6.guid,
        };joints.Add(j3);

       


        //CREATE modules consisting of 2 frames and 1 joint:
        List<Module> modules = new List<Module>();
        for (int i = 0; i<frames.Count;i++)
        {

            if(i % 2 != 0 )//each 2nd frame, we connect with joint in a module
            {
                var mod = new Module();
                modules.Add(mod);
                mod.Create(frames[i - 1],new Simulation_Core.Joint(), frames[i]);
            }
        }

        //CREATE the robot by adding modules:
        Robot robot_serialize = new Robot();
        foreach (Module module in modules)
        {
            robot_serialize.Add_Module(module);
        }*/

        //CREATE Terrain:
        Scene scene_serialize = new Scene()
        {
            guid = Guid.NewGuid(),
            heightmap = "Heightmap3",
            position = new Vector3(),
            materialName = "rock",
            restitution = 0.1f,
            friction = 0.9f,
            height = 10
        };

        //Add robot and scene to scenario:
        Scenario scenario_serialize = new Scenario()
        {
            robot = robot_serialize,
            scene = scene_serialize
        };

        //Add to XML file:
        Serialize(scenario_serialize);
        
    }

    public static void Serialize(object item)
    {
        XmlSerializer serializer = new XmlSerializer(item.GetType());
        StreamWriter writer = new StreamWriter("Scenario.xml");
        serializer.Serialize(writer.BaseStream, item);
        writer.Close();
    }

    public static T Deserialize<T>()
    {
        XmlSerializer serializer = new XmlSerializer(typeof(T));
        StreamReader reader = new StreamReader("Scenario.xml");
        T deserialized = (T)serializer.Deserialize(reader.BaseStream);
        reader.Close();
        return deserialized;
    }

    Robot robot;//Global for pos/rot update
    
    void Load_Robot(Robot robot)
    {
        //Initialize modules with joints and frames (+agx objects)
        foreach (Module mod in robot.modules)
        {
            foreach (Frame frame in mod.frames)
            {
                frame.Initialize();
                frameVis.Add(new Frame_Vis(frame.guid, frame.position));
            }
            mod.Initialize(mod.frames[0], mod.frames[1]);//calls Create_Hinge
            jointVis.Add(new Joint_Vis(mod.joint.guid));

        }
        robot.Initialize();//Locks modules together

        this.robot = robot;
    }

    Scene scene;
    public void Load_Scene(Scene scene)
    {
        //Initialize scene:
        scene.Create();
        Scene_Vis scene_vis = new Scene_Vis(scene.guid,scene.vertices,scene.triangles,scene.uvs,scene.position, Resources.Load("grass") as Texture2D);

        this.scene = scene;
    }


    Dictionary<Guid, GameObject> vis_obj = new Dictionary<Guid, GameObject>();
    List<Frame_Vis> frameVis = new List<Frame_Vis>();
    List<Joint_Vis> jointVis = new List<Joint_Vis>();
    float count = 0;
    
    
    void Set_Dynamics()
    {
        int mod_n = robot.modules.Count;

        Dynamics.amplitudes = new float[mod_n];
        Dynamics.period = new float[mod_n];
        Dynamics.phaseDiff = new float[mod_n];
        Dynamics.offset = new float[mod_n];

        for(int i = 0; i<robot.modules.Count; i++)
        {
            Dynamics.amplitudes[i] = 0;
            Dynamics.period[i] = 5;
            Dynamics.phaseDiff[i] = 0;
            Dynamics.offset[i] = 0;
        }


        float phaseOffset = 1.5f;
        for(int i = 0; i<mod_n; i++)
        {
            if (i != 0)
                Dynamics.phaseDiff[i] = Dynamics.phaseDiff[i - 1] + phaseOffset;
        }
    }

    void Update_AGX()
    {
        Agx_Simulation.StepForward();
        float offset = 0;

        Dynamics.Turn(robot, Time.fixedTime);//fixed time might have to be changed to a dt reset when program is changed. 

        foreach(Module module in robot.modules)
        {
            foreach (Frame frame in module.frames)
            {
                frame.Update();
                //Retrieves Frameobject with GUID, and updates position,size,rotation:
                try { frameVis.Find(x => x.guid == frame.guid).Update(frame.position, frame.size, frame.rotation); } catch { Debug.Log("Could not find frame with Guid."); }
            }        
            //Dynamics:
            //count += dt*0.2f;
            //Should instead be inside another class (this formula) and called from robot.controller(dt);
            //module.joint.MOVE(Mathf.Sin(offset + count * move));offset += 1.2f;

            

            //Joint:
            module.joint.Update();

            //Dyn:
            module.Update();

            try { jointVis.Find(x => x.guid == module.joint.guid).Update(module.joint.Vis_ContactPoints()); } catch { Debug.Log("Could not find joint with Guid." ); }

        }

        
    }

    void OnApplicationQuit()///When Unity closes, shutdown AgX.
    {
        Agx_Simulation.Stop();
        Debug.Log("Application ending after " + Time.time + " seconds");

    }

    void Plugins()
    {
        string dir = Directory.GetCurrentDirectory();
        string pathToAgX = Directory.GetParent(dir) + @"\AgX\AGX-2.21.1.2\";
        agxIO.Environment.instance().getFilePath(agxIO.Environment.Type.RESOURCE_PATH).pushbackPath(pathToAgX + @"\bin\x64\plugins");
        agxIO.Environment.instance().getFilePath(agxIO.Environment.Type.RUNTIME_PATH).pushbackPath(pathToAgX + @"\bin\x64\plugins");
        agxIO.Environment.instance().getFilePath(agxIO.Environment.Type.RESOURCE_PATH).pushbackPath(pathToAgX);
        Debug.Log(pathToAgX);
    }
    
}
