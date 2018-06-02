/* Simulation variables Core class (Simulation_Core)
 * Torstein Sundnes Lenerand
 * NTNU Ålesund
 */

using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;
using AgX_Interface;
using System.Xml.Serialization;
using System.Drawing;
using System.IO;


namespace Simulation_Core
{

    public class Scenario
    {
        public Robot robot;
        public Scene scene;
        public List<SceneObject> sceneObjects;
        public List<ContactFriction> contactFrictions;
    }

    [XmlRoot("Robot", Namespace = "Assembly")]
    public class Robot
    {
        public List<Module> modules = new List<Module>();
        public List<SensorModule> sensorModules = new List<SensorModule>();
        public List<Joint> locks = new List<Joint>();

        public string leftFrameDir, rightFrameDir;

        internal Vector3 position;

        public void Add_Module(Module module)
        {
            modules.Add(module);

            module.axis = module.frames[0].QuatToRot().x == 0 ? "Pitch" : "Yaw";
        }
        public void Add_Module(Module module, Joint lockjoint)
        {
            modules.Add(module);
            locks.Add(lockjoint);

            module.axis = module.frames[0].QuatToRot().x == 0 ? "Pitch" : "Yaw";
        }

        public void Add_SensorModule(SensorModule module, Joint r_lockjoint)
        {//This just creates the locks, doesnt need more info.
            sensorModules.Add(module);
            locks.Add(r_lockjoint);
        }
        public void Add_SensorModule(Joint l_lockjoint, SensorModule module, Joint r_lockjoint)
        {
            sensorModules.Add(module);
            locks.Add(l_lockjoint);
            locks.Add(r_lockjoint);
        }

        public void Initialize()//Initializes lock joints.
        {

            //AgX_Assembly.AddToSim();
            //Creates joints between frames and inits frame objects:
            foreach (Module mod in modules)
            {
                mod.Initialize();
            }
            foreach (SensorModule mod in sensorModules)
            {
                mod.Initialize();
            }

            int lockNrCount = 0;
            //Sets locks between modules:(ITERATES CURRENT MODULE)-> 
            for (int i = 0; i < modules.Count; i++)
            {

                if (sensorModules.Any(x => x.rightMod_Nr == i))//[]>X | if this module is to the right of a sensor moduleif(sensorModules.Any(x => x.rightMod_Nr == modules[i].number)
                {
                    locks[lockNrCount].Create_SensorModuleLock(sensorModules.Find(x => x.rightMod_Nr == i), modules[i].frames[0]);
                    lockNrCount++;

                    if (!sensorModules.Any(x => x.leftMod_Nr == i) && i + 1 < modules.Count)//X->X | if there is no sensorModule to the right, and there IS a module to the right
                    {
                        locks[lockNrCount].Create_Lock(modules[i].frames[1], modules[i + 1].frames[0]);
                        lockNrCount++;
                    }
                }

                if (sensorModules.Any(x => x.leftMod_Nr == i))//X<-[] | if this module is to the left of a sensor module:
                {
                    locks[lockNrCount].Create_SensorModuleLock(modules[i].frames[1], sensorModules.Find(x => x.leftMod_Nr == i));
                    lockNrCount++;
                }

                if (i + 1 < modules.Count && !sensorModules.Any(x => x.leftMod_Nr == i || x.rightMod_Nr == i))//If the iterator is not exceeding module count, and this module and has no right or left to any sensor
                {
                    locks[lockNrCount].Create_Lock(modules[i].frames[1], modules[i + 1].frames[0]);
                    lockNrCount++;
                }
            }

            AgX_Assembly.AddToSim();
        }

        public void Update()
        {
            position = Vector3.zero;
            foreach (Module mod in modules)
            {
                mod.Update();
                position += mod.position;
            }
            position /= modules.Count;
            foreach (SensorModule mod in sensorModules)
            {
                mod.Update();
                if (mod.forceSensor != null)
                {
                    mod.forceSensor.Update();
                }
            }
        }
        public void RemovePhysicsObjects()
        {
            AgX_Assembly.RemoveFromSim();

            /* foreach(Joint lockjoint in locks)
             {
                 if(lockjoint.agxJoint != null)
                 lockjoint.agxJoint.Remove();
                 lockjoint.agxJoint = null;
             }*/
            locks.Clear();
            locks = null;
            /* foreach (Module mod in modules)
             {
                 mod.joint.agxJoint.Remove();mod.joint.agxJoint = null;
                 mod.frames[0].agxFrame.Remove();mod.frames[0].agxFrame = null;
                 mod.frames[1].agxFrame.Remove();mod.frames[1].agxFrame = null;
             }*/
            modules.Clear();
            modules = null;
            /* foreach(SensorModule mod in sensorModules)
             {
                 mod.agxPrimitive.Remove();mod.agxPrimitive = null;
             }*/
            sensorModules.Clear();
            sensorModules = null;
        }
    }


    public class Module
    {
        public int mod_Nr;
        public Vector3 position;
        public string axis;
        public Frame[] frames = new Frame[2];
        public Joint joint;

        //public double z_leftEdge, z_rightEdge, top, bot;

        public void Create(Frame left, Joint joint, Frame right)//Creates are for Scene designer, Initialize is for simulator
        {
            frames[0] = left; frames[1] = right;
            this.joint = joint;
            position = frames[0].position;
        }

        public void Initialize()
        {
            foreach (Frame frame in frames)
                frame.Initialize();

            position = Vector3.Lerp(frames[0].position, frames[1].position, 0.5f);
            joint.Create_Hinge(frames[0], frames[1]);
        }

        public void Update()
        {
            foreach (Frame frame in frames)//Update all frames in the module
            {
                frame.Update();
            }

            //Update Joint: (There is always a joint)
            joint.Update();

            //Update module position
            position = Vector3.Lerp(frames[0].position, frames[1].position, 0.5f);
        }
    }




    public class Frame
    {
        public Guid guid;
        public string shape;

        public double scale;
        public Vector3 position;
        public Vector3 rotation;
        public Quaternion quatRotation;
        public double mass;
        public bool isStatic;
        public string materialName;

        public Vector3[] meshVertices; public Vector2[] meshUvs; public int[] meshTriangles;
        internal AgX_Frame agxFrame;

        public void Initialize() //Create frame object
        {
            ScaleMesh();
            QuatToRot();
            agxFrame = new AgX_Frame(this.guid, shape, meshVertices, meshUvs, meshTriangles, scale, position, quatRotation, mass, isStatic, materialName);
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
            position = agxFrame.GetPosition();
            //rotation = agxFrame.Get_Rotation();
            quatRotation = agxFrame.GetQuatRotation();
        }
        public Quaternion GetQuatRot()
        {
            return quatRotation;
        }
        public void SetMesh(Vector3[] vertices, Vector2[] uvs, int[] triangles)
        {
            meshVertices = vertices;
            meshUvs = uvs;
            meshTriangles = triangles;
        }
        public Vector3 QuatToRot()
        {
            rotation = quatRotation.ToEulerRad();
            return quatRotation.ToEulerRad();
        }
    }


    public class Joint
    {
        public Guid guid, leftFrameGuid, rightFrameGuid;
        public string type;
        public double lowerRangeLimit, upperRangeLimit;

        /*Alternative:*/
        public double Kp = 3;
        public double max_vel;

        internal AgX_Joint agxJoint;
        internal Frame left, right;

        public void Create_Hinge(Frame left, Frame right)
        {
            this.left = left; this.right = right;
            type = "Hinge";
            agxJoint = new AgX_Joint(guid);

            //Add joint between top and bottom frame
            agxJoint.Create_Hinge("Hinge", left.agxFrame, right.agxFrame, lowerRangeLimit, upperRangeLimit);
            agxJoint.AddtoAssembly();
        }
        public void Create_Lock(Frame left, Frame right)
        {
            this.left = left; this.right = right;
            type = "Lock";
            agxJoint = new AgX_Joint(guid);
            agxJoint.Create_Lock("Lock", left.agxFrame, right.agxFrame);
            agxJoint.AddtoAssembly();
        }

        /*------------------------------------------------Sensory Modules:------------------------------------------------*/
        public void Create_SensorModuleLock(Frame right, SensorModule s_mod)//sensor placed last, right frame of left module 
        {
            agxJoint = new AgX_Joint(guid);
            agxJoint.Create_Lock("Lock", right.agxFrame, s_mod.agxPrimitive);//FIX IN AGX INTERFACE
            agxJoint.AddtoAssembly();
        }
        public void Create_SensorModuleLock(SensorModule s_mod, Frame left)//sensor placed first, left frame of right module
        {
            agxJoint = new AgX_Joint(guid);
            agxJoint.Create_Lock("Lock", s_mod.agxPrimitive, left.agxFrame);//FIX IN AGX INTERFACE
            agxJoint.AddtoAssembly();
        }

        /*-------------------------------------------------Force Sensors:-------------------------------------------------*/
        public void Create_ForceSensorLock(SensorModule sm, ForceSensor fs, Vector3 lockPosition)
        {
            agxJoint = new AgX_Joint(guid);
            agxJoint.ForceSensorLock(sm.agxPrimitive, fs.agxSensor, lockPosition);
            agxJoint.AddtoAssembly();
        }
        public void Create_DistanceSensorLock(SensorModule sm, DistanceSensor ds, Vector3 lockPosition)
        {
            agxJoint = new AgX_Joint(guid);
            agxJoint.DistanceSensorLock(sm.agxPrimitive, ds.agxPrimitive, lockPosition);
            agxJoint.AddtoAssembly();
        }

        public void SetAngle(double requested_angle)
        {
            double error = requested_angle - agxJoint.GetAngle();//Expand to pid if required later?

            if (Kp * error > max_vel)
                agxJoint.SetSpeed(max_vel);
            else if (Kp * error < -max_vel)
                agxJoint.SetSpeed(-max_vel);
            else
                agxJoint.SetSpeed(Kp * error);
        }

        public void Stabilize_Angle()//Resets the joint movement
        {
            double error = 0 - agxJoint.GetAngle();

            if (Kp * error > 0.5f)
                agxJoint.SetSpeed(0.5f);
            else if (Kp * error < -0.5f)
                agxJoint.SetSpeed(-0.5f);
            else
                agxJoint.SetSpeed(Kp * error);
        }

        public double GetAngle()
        {
            return agxJoint.GetAngle();
        }

        public void Update()
        {
            //mid_Position = left.position - left.GetQuatRot() * Vector3.forward;// * left.scale.z;
        }
    }




    public class SensorModule //Square
    {
        public Guid guid;
        public int leftMod_Nr, rightMod_Nr;

        //Force sensor:
        public ForceSensor forceSensor;
        //ForceSensor locks:
        public Joint forceSensorLock;
        //Force sensor position:
        public Vector3 forceLockPosition;

        //Distance sensor lists:
        public List<DistanceSensor> distanceSensors = new List<DistanceSensor>();
        //Distance sensor positions:
        public Vector3 distanceLockPosition;
        //Distance sensor locks:
        public List<Joint> distanceSensorLocks = new List<Joint>();

        public Vector3 position;
        public Vector3 rotation;
        public Vector3 size;
        public Quaternion quatRotation;
        public double mass;
        public string materialName;

        internal AgX_Primitive agxPrimitive;

        public void Initialize()
        {
            agxPrimitive = new AgX_Primitive(guid, "Box", position, quatRotation, size, mass, materialName, false, true);
            if (forceSensor != null)
            {
                forceSensor.Initialize();
                forceSensorLock.Create_ForceSensorLock(this, forceSensor, forceLockPosition);
                forceSensor.fs_Joint = forceSensorLock.agxJoint;//Sets the sensor's lock (need for calculating forces)
            }

            //distance
            int count = 0;
            foreach (DistanceSensor ds in distanceSensors)
            {
                ds.Initialize();

                distanceSensorLocks[count].Create_DistanceSensorLock(this, ds, distanceLockPosition);
                count++;
            }
        }

        public void ConnectForceSensor(ForceSensor fs)//Pre-initialize
        {
            forceSensor = fs;
            forceSensorLock = new Joint();
            switch (fs.sensorPosition)
            {
                case 0:
                    {
                        fs.position = position; fs.position.y = position.y - size.y - fs.size.y / 2 - 0.001;//bot position
                        forceLockPosition = fs.position; forceLockPosition.y = fs.position.y + fs.size.y / 2 + 0.0005;//position of the point where they are locked together.
                        break;//
                    }
                case 1:
                    {
                        fs.position = position; fs.position.x = position.x - size.x - fs.size.x / 2 - 0.001;//left position
                        forceLockPosition = fs.position; forceLockPosition.x = fs.position.x + fs.size.x / 2 + 0.0005;//position of the point where they are locked together.
                        break;
                    }
                case 2:
                    {
                        fs.position = position; fs.position.y = position.y + size.y + fs.size.y / 2 + 0.001;//top position
                        forceLockPosition = fs.position; forceLockPosition.y = fs.position.y - fs.size.y / 2 - 0.0005;//position of the point where they are locked together.
                        break;
                    }
                case 3:
                    {
                        fs.position = position; fs.position.x = position.x + size.x + fs.size.x / 2 + 0.001;//right position
                        forceLockPosition = fs.position; forceLockPosition.x = fs.position.x - fs.size.x / 2 - 0.0005;//position of the point where they are locked together.
                        break;
                    }
            }
        }
        public void ConnectDistanceSensors(List<DistanceSensor> distSensors)
        {
            distanceSensors = new List<DistanceSensor>();
            distanceSensorLocks = new List<Joint>();
            foreach (DistanceSensor ds in distSensors)
            {

                distanceSensorLocks.Add(new Joint());
                distanceSensors.Add(ds);
                ds.position = distanceLockPosition = position; //UnityEngine.Debug.Log("Working");
                switch (ds.sensorPosition)
                {
                    case 0:
                        {
                            ds.ray_Direction = new Vector3(0, -1, 0);
                            break;
                        }
                    case 1:
                        {
                            ds.ray_Direction = new Vector3(-1, 0, 0);
                            break;
                        }
                    case 2:
                        {
                            ds.ray_Direction = new Vector3(0, 1, 0);
                            break;
                        }
                    case 3:
                        {
                            ds.ray_Direction = new Vector3(1, 0, 0);
                            break;
                        }
                    case 4:
                        {
                            ds.ray_Direction = new Vector3(0, 0, 1);
                            break;
                        }
                    case 5:
                        {
                            ds.ray_Direction = new Vector3(0, 0, -1);
                            break;
                        }
                }

            }
        }

        public void Update()
        {
            position = agxPrimitive.GetPosition();
            //rotation = agxPrimitive.Get_Rotation();
            quatRotation = agxPrimitive.GetRotation();
            if (forceSensor != null)
            {
                forceSensor.Update();
            }
            if (distanceSensors != null)
                foreach (DistanceSensor ds in distanceSensors)
                    ds.Update();
        }

        public Vector3 QuatToRot()
        {
            rotation = quatRotation.ToEulerRad();
            return quatRotation.ToEulerRad();
        }
    }


    public class ForceSensor
    {
        public Guid guid;
        public double forceValue;//private?get?
        public int sensorPosition;//0 = below, 1 = left, 2 = top, 3 = right;
        public Vector3 position;
        public Quaternion rotation;
        public string materialName;
        public double mass;
        public Vector3 size = new Vector3(0.1, 0.01, 0.04);//??

        internal AgX_Joint fs_Joint;
        internal AgX_ForceSensor agxSensor;

        public void Initialize()
        {
            agxSensor = new AgX_ForceSensor(guid, materialName, position, rotation, size, mass);

        }

        public void Update()
        {
            position = agxSensor.GetPosition();
            rotation = agxSensor.GetRotation();
            forceValue = fs_Joint.GetForce();
        }
    }

    public class DistanceSensor
    {
        public Guid guid;
        public int sensorPosition;//0 = below, 1 = left, 2 = top, 3 = right;
        public Vector3 position;
        public Vector3 rotation;
        public Quaternion quatRotation;
        public double mass;
        public Vector3 size = new Vector3(0.01, 0.01, 0.01);//??

        public double max_rayDistance = 10;

        private double hit_distance;

        public Vector3 ray_Direction;//Rotation?
        public double ray_Resolution = 0.1;

        internal AgX_Primitive agxPrimitive;

        public void Initialize()
        {
            agxPrimitive = new AgX_Primitive(guid, "Box", position, quatRotation, size, mass, "Plastic", false, true);

        }

        public void Update()
        {
            position = agxPrimitive.GetPosition();
            quatRotation = agxPrimitive.GetRotation();
        }
        public double GetSensorDistance()
        {
            return hit_distance;
        }

        public void CalculateDistance(List<SceneObject> sceneObjects)
        {
            //Initialize with the max value 
            hit_distance = max_rayDistance;
            double distance = 0;
            bool objectHit = false;

            // shoot ray a certrain distance, the direction of ray_direction. 
            //Direction
            var direction = quatRotation * ray_Direction;//UnityEngine.Debug.Log(direction.x + "," + direction.y + "," + direction.z);

            while (distance < max_rayDistance && objectHit == false)
            {
                //iterate
                distance += ray_Resolution;

                //New position of the ray
                var RayPosition = position + direction * (float)distance;

                //check if any objects intersect
                foreach (SceneObject obj in sceneObjects)
                {
                    if (RayHitInside(obj, RayPosition))
                    {
                        hit_distance = distance;
                        objectHit = true;//We do not need to check if its the closest object, because the distance variable ensures it is.
                    }
                }
            }


        }

        Boolean RayHitInside(SceneObject obj, Vector3 raypos)
        {

            if (obj.shape == "Box")//check boxes
            {
                double xmin = obj.position.x - obj.size.x;
                double xmax = obj.position.x + obj.size.x;
                double ymin = obj.position.y - obj.size.y;
                double ymax = obj.position.y + obj.size.y;
                double zmin = obj.position.z - obj.size.z;
                double zmax = obj.position.z + obj.size.z;


                if (raypos.x < xmin || raypos.y < ymin || raypos.z < zmin || raypos.x > xmax || raypos.y > ymax || raypos.z > zmax)//if point is outside
                {
                    return false;
                }
            }
            if (obj.shape == "Sphere")//check spheres
            {
                var x = Math.Pow((raypos.x - obj.position.x), 2);
                var y = Math.Pow((raypos.y - obj.position.y), 2);
                var z = Math.Pow((raypos.z - obj.position.z), 2);
                var radius = Math.Pow(((obj.size.x + obj.size.y + obj.size.z) / 3), 2);


                if (x + y + z > radius)//if point is outside
                {
                    return false;
                }
            }

            //if it is not outside
            return true;
        }
    }

    public class SceneObject
    {
        public Guid guid;
        public Vector3 size, position, rotation;
        public Quaternion quatRotation;
        public string materialName, shape;
        public double mass;
        public bool isStatic;

        internal AgX_Primitive agxPrimitive;

        public void Initialize()
        {
            agxPrimitive = new AgX_Primitive(guid, shape, position, quatRotation, size, mass, materialName, isStatic, false);
        }

        public void Update()
        {
            position = agxPrimitive.GetPosition();
            //rotation = agxPrimitive.Get_Rotation();
            quatRotation = agxPrimitive.GetRotation();
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
        public double height;
        //Water
        //Air

        AgX_Scene scene;
        public void Create()
        {
            //position.y += height/2;
            LoadTerrainFromImage();//Loads the heightmap

            scene = new AgX_Scene(guid, vertices, triangles, position, materialName);

        }

        private void LoadTerrainFromImage()

        {
            // Modified from: https://answers.unity.com/questions/1033085/heightmap-to-mesh.html
            //Unity:
            /*Texture2D heightMap = new Texture2D(250, 250);
            heightMap.LoadImage(Convert.FromBase64String(height_Image));*/

            //C#,Visual Studio
            Bitmap heightMap = new Bitmap(250, 250);
            using (var ms = new MemoryStream(Convert.FromBase64String(height_Image)))//C#/Visual Studio
            {
                heightMap = new Bitmap(ms);
            }

            //Bottom left section of the map, other sections are similar
            for (int i = 0; i < 250; i++)
            {
                for (int j = 0; j < 250; j++) //OUTER PIXELS MUST BE 0, fix
                {
                    //Add each new vertex in the plane
                    if (i < heightMap.Width && j < heightMap.Height)
                    {
                        vertices.Add(new Vector3(i, heightMap.GetPixel(i, j).R * height / 255.0f, j));// dibide by 255 to get the normalized size, and multiply by height
                    }
                    else
                        vertices.Add(new Vector3(i, 0, j));// dibide by 255 to get the normalized size, and multiply by height

                    //Skip if a new square on the plane hasn't been formed
                    if (i == 0 || j == 0 || i == 249 || j == 249) continue;
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

        public void CreateMesh()
        {
            LoadTerrainFromImage();
        }

    }


    public class ContactFriction
    {
        public string material1, material2;
        public double restitution, friction, youngsModulus;
    }
}
