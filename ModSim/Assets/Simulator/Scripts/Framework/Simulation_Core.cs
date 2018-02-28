using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using AgX_Interface;
using System.Xml.Serialization;
//using UnityEngine.UI;

namespace Simulation_Core
{
    public class Scenario
    {
        public Robot robot;
        public Scene scene;
    }


    [XmlRoot("Robot", Namespace = "Assembly")]
    public class Robot
    {
        public List<Module> modules = new List<Module>();
        public List<Joint> locks = new List<Joint>();

        public void Add_Module(Module module)
        {
            modules.Add(module);
        }
        public void Add_Module(Module module, Joint lockjoint)
        {
            modules.Add(module);
            locks.Add(lockjoint);
        }

        public void Initialize()//Initializes lock joints.
        {
            //Set locks
            for(int i = 0; i<modules.Count-1; i++)
            {
                locks[i].Create_Lock(modules[i].frames[1], modules[i + 1].frames[0]);
                
            }
            //Set Pitch or Yaw
            foreach (Module module in modules)
                module.Axis = module.frames[0].rotation.z == 0 ? "Pitch" : "Yaw";
            
        }
        void Control()//be changed by the user
        {
            
            //movement.roll() //form another class the user can manipulate.(add new functionality by string script etc.) 
            //roll();
            //Custom(params);

        }
        void Roll()
        {
            //one for loop for pitch,

            //one for yaw
        }
        void Jumping()
        {

        }
    }

    
    public class Module
    {
        public Vector3 position;
        public string Axis;
        public Frame[] frames = new Frame[2];
        public Joint joint;
        

        public void Create(Frame left,Joint joint, Frame Right)//Creates are for Scecne designer, Initialize is for simulator
        {
            frames[0] = left; frames[1] = Right;
            this.joint = joint;
        }
        public void Initialize(Frame left, Frame right)
        {
            position = Vector3.Lerp(left.position, right.position,0.5f);
            joint.Create_Hinge(left, right);
        }
        public void Update()
        {
            position = Vector3.Lerp(frames[0].position, frames[1].position, 0.5f);
        }
    }
    

    public class Frame
    {
        public Guid guid;
        public string shape;
        public Vector3[] meshVertices; public Vector2[] meshUvs; public int[] meshTriangles;
        public float scale;
        public Vector3 position;
        public Vector3 rotation;
        private Quaternion quatRotation;
        public float mass;
        public Boolean isStatic;
        public string materialName;

        internal AgX_Frame frame;

        public void Initialize() //Create frame object
        {
            ScaleMesh();
            frame = new AgX_Frame(this.guid, shape,meshVertices, meshUvs, meshTriangles, scale,position,rotation,mass,isStatic,materialName);
        }

        private void ScaleMesh()
        {
            Vector3[] tmp_Vertices = meshVertices;
            for (int i = 0; i < tmp_Vertices.Length; i++)
            {
                tmp_Vertices[i].x *= scale;
                tmp_Vertices[i].y *= scale;
                tmp_Vertices[i].z *= scale;
            }
            meshVertices = tmp_Vertices;
        }

        public void Update()//Update variables
        {
            //scale = frame.Get_Size();
            position = frame.Get_Position();
            rotation = frame.Get_Rotation();
            quatRotation = frame.Get_QuatRotation();
        }
        public Quaternion GetQuatRot()
        {
            return quatRotation;
        }
        public void setMesh(Vector3[] vertices, Vector2[] uvs, int[] triangles)
        {
            meshVertices = vertices;
            meshUvs = uvs;
            meshTriangles = triangles;
        }
    }


    public class Joint
    {
        public Guid guid, leftFrameGuid, rightFrameGuid;
        public string type;
        public Vector3 mid_Position;
        public float leftRangeLimit, rightRangeLimit;

        /*Alternative:*/
        public float Kp = 1;
        public float max_vel;

        internal AgX_Joint joint;
        internal Frame left, right;

        public void Create_Hinge(Frame left, Frame right)
        {
            this.left = left; this.right = right;
            type = "Hinge";
            joint = new AgX_Joint(guid);

            //Add joint between top and bottom frame
            joint.Create_Hinge("Hinge", left, right, leftRangeLimit, rightRangeLimit);
            joint.AddToSim();
        }
        public void Create_Lock(Frame left, Frame right)
        {
            this.left = left; this.right = right;
            type = "Lock";
            joint = new AgX_Joint(guid);
            joint.Create_Lock("Lock", left, right);
            joint.AddToSim();
        }

        public void MOVE(float requested_angle)
        {
            float error = requested_angle - joint.Get_Angle();//Expand to pid if required later?

            if (Kp * error > max_vel)
                joint.Set_Speed(max_vel);
            else if (Kp * error < -max_vel)
                joint.Set_Speed(-max_vel);
            else
                joint.Set_Speed(Kp * error);
        }

        public void Reset_Angle()//Resets the joint movement
        {
            float error = 0 - joint.Get_Angle();

            if (Kp * error > 0.5f)
                joint.Set_Speed(0.5f);
            else if (Kp * error < -0.5f)
                joint.Set_Speed(-0.5f);
            else
                joint.Set_Speed(Kp * error);
        }

        public void Update()
        {
            mid_Position = left.position - left.GetQuatRot() * Vector3.forward;// * left.scale.z;
        }

        public Vector3[] Vis_ContactPoints()
        {
            Vector3 p_left = left.position; //left.position.x + Math.Sign((left.position - this.position).magnitude) * left.size;

            mid_Position = left.position - left.GetQuatRot() * Vector3.forward;// *left.scale.z ;//Returns the position of lock
            //Vector3 p_right = new Vector3(right.position.x, right.position.y, right.position.z + right.size.z / 2);
            Vector3 p_right = right.position;//insert angle insteadd of 45

            return new Vector3[3]{p_left,mid_Position,p_right};
        }
    }


    public class Scene
    {
        public Guid guid;
        public string height_Image;

        public List<Vector3> vertices = new List<Vector3>();
        public List<int> triangles = new List<int>();
        public Vector2[] uvs;

        public Vector3 position;
        public string materialName;
        public float height;
        //Water
        //Air

        Agx_Scene scene;
        public void Create()
        {
            TerrainFromImage();//Loads the heightmap

            scene = new Agx_Scene(guid, vertices, triangles, position, materialName, height);

        }

        private void TerrainFromImage()

        {
            // Modified from: https://answers.unity.com/questions/1033085/heightmap-to-mesh.html
            Texture2D heightMap = new Texture2D(250,250);
            heightMap.LoadImage(Convert.FromBase64String(height_Image));

            //Bottom left section of the map, other sections are similar
            for (int i = 0; i < 250; i++)
            {
                for (int j = 0; j < 250; j++) //OUTER PIXELS MUST BE 0, fix
                {
                    //Add each new vertex in the plane
                    vertices.Add(new Vector3(i, heightMap.GetPixel(i, j).grayscale * height, j));
                    //Skip if a new square on the plane hasn't been formed
                    if (i == 0 || j == 0) continue;
                    //Adds the index of the three vertices in order to make up each of the two tris
                    triangles.Add(250 * i + j); //Top right
                    triangles.Add(250 * i + j - 1); //Bottom right
                    triangles.Add(250 * (i - 1) + j - 1); //Bottom left - First triangle
                    triangles.Add(250 * (i - 1) + j - 1); //Bottom left 
                    triangles.Add(250 * (i - 1) + j); //Top left
                    triangles.Add(250 * i + j); //Top right - Second triangle
                }
            }

            uvs = new Vector2[vertices.Count];
            for (var i = 0; i < uvs.Length; i++) //Give UV coords X,Z world coords
                uvs[i] = new Vector2(vertices[i].x, vertices[i].z);

        }

        void Add()
        {
            //scenario.Add_Rb();
        }
    }

    public class Sensor
    {

        class Force
        {
            Guid guid;
            float value; //sensory value (Force)
            Vector3 position;
            internal AgX_Sensor sensor;
             void Initialize()
            {
                sensor = new AgX_Sensor(guid,"plastic",position,1.0f);
            }
        }
        class Distance
        {

        }

    }
	
}
