using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using AgX_Interface;
using Simulation_Core;
using Unity_Visualization;
using System;
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

        SetContactPoints();

        Serialization();//Move this to Scene designer

        //LOAD:
        Scenario scenario = Deserialize<Scenario>();
        Load_Robot(scenario.robot);
        Load_Scene(scenario.scene);
        
        InvokeRepeating("Update_AGX", 0, dt);
        
        

    }

    void SetContactPoints()
    {
        Agx_Simulation.AddContactMaterial("Plastic","Rock",0.4f,0.3f, (float)3.654E9);
    }

    void Serialization()
    {
        Robot robot_serialize = new Robot();

        List<Frame> frames = new List<Frame>();
        List<Simulation_Core.Joint> joints = new List<Simulation_Core.Joint>();

        Vector3 frame1Pos = new Vector3(15, 12, 40);
        Vector3 frame2Pos = new Vector3(15, 12, 40);

    for(int i = 0; i < 10 ; i++)
    {

            var f1 = new Frame()//test create new object
            {
                guid = Guid.NewGuid(),
                shape = "Box",
                position = new Vector3(frame1Pos.x, frame1Pos.y, frame1Pos.z - 1.6f * i),// new Vector3(15, 15, 20 -6*i),//20 = start position
                scale = 10,
                /*rotation = Vector3.zero,//*/
                rotation =  i % 2 == 0 ? new Vector3(-Mathf.PI / 2, 0, 0) : new Vector3(-Mathf.PI / 2, 0, Mathf.PI / 2),//pitch yaw
                mass = 50,
                isStatic = false,
                materialName = "Plastic"
            };

            var f2 = new Frame()//test create new object
            {
                guid = Guid.NewGuid(),
                shape = "Box",
                position = new Vector3(frame2Pos.x, frame2Pos.y, frame2Pos.z - 1.6f * i),// new Vector3(15, 15, 16-6*i),
                scale = 10,
                /*rotation = Vector3.zero,//*/
                rotation =  i % 2 == 0 ? new Vector3(-Mathf.PI / 2, 0, 0) : new Vector3(-Mathf.PI/2, 0, Mathf.PI / 2),
                mass = 50,
                isStatic = false,
                materialName = "Plastic"
            };

            var j1 = new Simulation_Core.Joint()
            {
                guid = Guid.NewGuid(),
                leftFrameGuid = f1.guid,
                rightFrameGuid = f2.guid,
                type = "Hinge",
                leftRangeLimit = -(float)Math.PI / 2, rightRangeLimit = (float)Math.PI / 2,
                max_vel = 20.0f
                
            };

            var module = new Module();
            module.Create(f1, j1, f2);

            //Create sensor here, on each left of module (and one on right). 
            
            if(i > 0)
            {
                robot_serialize.Add_Module(module, new Simulation_Core.Joint());
            }else
                robot_serialize.Add_Module(module/*,lockJoint*/);
            
    }

        //CREATE Terrain:
        Texture2D hMap = Resources.Load("Heightmap3") as Texture2D;//Rename to terrain
        byte[] bytes = hMap.EncodeToPNG();
        
        
        Scene scene_serialize = new Scene()
        {
            guid = Guid.NewGuid(),
            height_Image = Convert.ToBase64String(bytes),
            position = new Vector3(),
            materialName = "Rock",
            height = 10
        };
        Debug.Log(bytes.Length);

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
    
    void Load_Robot(Robot robot)
    {
        //Initialize modules with joints and frames (+agx objects) : SHOULD BE IN SCENE DESIGNER, send triangles, verts and uvs!
        string dir = Application.streamingAssetsPath + "/Robot/";
        Mesh leftMesh = new Mesh();
        Mesh rightMesh = new Mesh();
        ObjImporter import = new ObjImporter();
        leftMesh = import.ImportFile(dir + "upper.obj");Bounds leftBound = leftMesh.bounds;
        rightMesh = import.ImportFile(dir + "bottom.obj");Bounds rightBound = rightMesh.bounds;

        Vector3 start = new Vector3(15, 12, 40);
        //new z pos is start.z - meshLength*i. 
        foreach (Module mod in robot.modules)
        {
            mod.frames[0].setMesh(leftMesh.vertices, leftMesh.uv, leftMesh.triangles); mod.frames[1].setMesh(rightMesh.vertices, rightMesh.uv, rightMesh.triangles);

            
            foreach (Frame frame in mod.frames)
            {
                //SHOULD here set position of each frame, based on mesh size, etc. 
                
                frame.position = start;
                frame.Initialize();
            }
            //Position modules:
            float leftPos = start.z + (mod.frames[0].scale * -leftBound.min.y);//y is z before they are rotated in the scene
            float rightPos = start.z + (mod.frames[0].scale * -rightBound.max.y);
            start.z = start.z - (leftPos - rightPos) - 0.001f;

            mod.Initialize(mod.frames[0], mod.frames[1]);//calls Create_Hinge

            Mesh l = new Mesh() { vertices = mod.frames[0].meshVertices, uv = mod.frames[0].meshUvs, triangles = mod.frames[0].meshTriangles };
            Mesh r = new Mesh() { vertices = mod.frames[1].meshVertices, uv = mod.frames[1].meshUvs, triangles = mod.frames[1].meshTriangles };

            frameVis.Add(new Frame_Vis(mod.frames[0].guid, l, mod.frames[0].position));
            frameVis.Add(new Frame_Vis(mod.frames[1].guid, r, mod.frames[1].position));

            //jointVis.Add(new Joint_Vis(mod.joint.guid));

        }
        robot.Initialize();//Locks modules together

        this.robot = robot;
    }

    Scene scene;
    public void Load_Scene(Scene scene)
    {
        //Initialize scene:
        scene.Create();
        Scene_Vis scene_vis = new Scene_Vis(scene.guid,scene.vertices,scene.triangles,scene.uvs,scene.position, Resources.Load("grass") as Texture);
        this.scene = scene;
    }


    List<Frame_Vis> frameVis = new List<Frame_Vis>();
    List<Joint_Vis> jointVis = new List<Joint_Vis>();
    


    void Update_AGX()
    {
        Agx_Simulation.StepForward();

        if (Time.fixedTime >= 2)//Wait for robot to settle on terrain
            if(!Dynamics.Control(robot, Time.fixedTime))//Movement
                Debug.Log("wrong command");

        foreach (Module module in robot.modules)
        {
            foreach (Frame frame in module.frames)
            {
                frame.Update();
                //Retrieves Frameobject with GUID, and updates position,size,rotation:
                try { frameVis.Find(x => x.guid == frame.guid).Update(frame.position/*, frame.scale*/, frame.rotation); } catch(NullReferenceException e) { Debug.Log("Could not find frame with Guid." + e); }
            }

            //Joint:
            module.joint.Update();

            //Dyn:
            module.Update();

            //try { jointVis.Find(x => x.guid == module.joint.guid).Update(module.joint.Vis_ContactPoints()); } catch(NullReferenceException e) { Debug.Log("Could not find joint with Guid." + e ); }

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
