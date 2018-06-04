using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Xml.Serialization;
using AgX_Interface;
using Simulation_Core;
using System;

public class startSim : MonoBehaviour {

    // Use this for initialization
    Robot robot = new Robot();
    SceneObject sceneobj;
    void Start()
    {
        //Start simulation:
        Agx_Simulation.Start(0.01);

        //Setting position and rotations:
        var pos = new AgX_Interface.Vector3(0, 0, 0);
        var u_quat = UnityEngine.Quaternion.Euler(0, 90, 0);
        AgX_Interface.Quaternion frame_rot = new AgX_Interface.Quaternion(u_quat.x, u_quat.y, u_quat.z, u_quat.w);

        //Create Frames:
        Frame[] frames = new Frame[2];
        for (int i = 0; i < 2; i++)
        {
            frames[i] = new Frame()
            {
                guid = Guid.NewGuid(),
                position = pos,
                scale = 10,
                quatRotation = frame_rot,
                //rotation = rot,
                mass = 10,
                isStatic = false,
                materialName = "Plastic"
            };
        }

        //download mesh obj file:
        loadmesh();
        
        //Set mesh of frames:
        frames[0].SetMesh(AgxHelper(leftmesh.vertices), AgxHelper(leftmesh.uv), leftmesh.triangles);
        frames[1].SetMesh(AgxHelper(rightmesh.vertices), AgxHelper(rightmesh.uv), rightmesh.triangles);
        
        //Create Joint:
        Simulation_Core.Joint joint = new Simulation_Core.Joint()
        {
            guid = Guid.NewGuid(),
            leftFrameGuid = frames[0].guid,
            rightFrameGuid = frames[1].guid,
            type = "Hinge",
            lowerRangeLimit = -Math.PI/2,
            upperRangeLimit = Math.PI/2,
            max_vel = Math.PI/6,
            Kp = 3
        };

        //Create Module:
        var module = new Module();
        module.Create(frames[0],joint,frames[1]);

        //Add module to robot:
        robot.Add_Module(module);

        //Add second module to robot:
        //robot.Add_Module(module2,new Simulation_Core.Joint());

        //Initialize robot:
        robot.Initialize();

        //Scene object:
        sceneobj = new SceneObject()
        {
            guid = Guid.NewGuid(),
            shape = "Box",
            size = new AgX_Interface.Vector3(5,1,10),
            position = new AgX_Interface.Vector3(0,-2,0),
            quatRotation = new AgX_Interface.Quaternion(0,0,0,1),
            materialName = "Rock",
            mass = 10,
            isStatic = true
        };
        sceneobj.Initialize();
        
        //Load vis from mesh and robot + scene object:
        Load_Vis();

        //Start sim update loop:
        InvokeRepeating("Update_Sim", 0.01f, 0.01f);
    }

    //vis
    GameObject sceneobjvis;
    GameObject[] FrameDemoVis = new GameObject[2];

    // Update is called once per frame
    void Update_Sim ()
    {
        //Update physics:
        Agx_Simulation.StepForward();
        //Update the robot:
        robot.Update();
        //Update the visualization:
        {
            //Frames:
            FrameDemoVis[0].transform.position = AgxHelper(robot.modules[0].frames[0].position);
            FrameDemoVis[0].transform.rotation = (AgxHelper(robot.modules[0].frames[0].quatRotation));
            FrameDemoVis[1].transform.position = AgxHelper(robot.modules[0].frames[1].position);
            FrameDemoVis[1].transform.rotation = (AgxHelper(robot.modules[0].frames[1].quatRotation));
            //Scene object:
            sceneobjvis.transform.position = AgxHelper(sceneobj.position);
        }
        robot.modules[0].joint.SetAngle(2);
	}
    Mesh leftmesh, rightmesh;
    void loadmesh()
    {
        ObjImporter import = new ObjImporter();
        leftmesh = import.ImportFile(Application.streamingAssetsPath + "/upper.obj");
        rightmesh = import.ImportFile(Application.streamingAssetsPath + "/bottom.obj");
    }

    void Load_Vis()
    {
        //Set frame meshes:
        FrameDemoVis[0] = GameObject.CreatePrimitive(PrimitiveType.Cube);
        FrameDemoVis[1] = GameObject.CreatePrimitive(PrimitiveType.Cube);

        Mesh[] mesh = new Mesh[2];
        mesh[0] = FrameDemoVis[0].GetComponent<MeshFilter>().mesh;
        mesh[1] = FrameDemoVis[1].GetComponent<MeshFilter>().mesh;

        var robotsize = new UnityEngine.Vector3(10,10,10);
        for(int i = 0; i<2;i++)
        {
            mesh[i].vertices = AgxHelper(robot.modules[0].frames[i].meshVertices);
            mesh[i].uv = AgxHelper(robot.modules[0].frames[i].meshUvs);
            mesh[i].triangles = robot.modules[0].frames[i].meshTriangles;
        }
        //Scene object
        sceneobjvis = GameObject.CreatePrimitive(PrimitiveType.Cube);
        sceneobjvis.transform.localScale = AgxHelper(sceneobj.size * 2);
        sceneobjvis.transform.position = AgxHelper(sceneobj.position);
    }


    //Helpers:

        UnityEngine.Vector3 AgxHelper(AgX_Interface.Vector3 vec)
    {
        var vector = new UnityEngine.Vector3();
        vector.x = (float)vec.x;
        vector.y = (float)vec.y;
        vector.z = (float)vec.z;

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

    UnityEngine.Vector3[] AgxHelper(AgX_Interface.Vector3[] vec)//THESE?? wrong?
    {
        var vectors = new UnityEngine.Vector3[vec.Length];

        for (int i = 0; i < vec.Length; i++)
        {
            vectors[i].x = (float)vec[i].x;
            vectors[i].y = (float)vec[i].y;
            vectors[i].z = (float)vec[i].z;
        }
        return vectors;
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
