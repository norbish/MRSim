/*from Algoryx and Unity To Object class(AUTOclass)
 * Torstein Sundnes Lenerand
 * NTNU Ålesund
 */

 ///This class contains the AgX object. 
 ///Use the Get methods for accessing the object in the main program. 

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using Simulation_Core;

namespace AgX_Interface
{
    public class Robot_Assembly
    {
        ///Modules in the robot
        List<Module_Assembly> modules;

        public Robot_Assembly(int count)//Adds new modules to the assembly
        {
            for (int i = 0; i < count; i++)
            {
                modules.Add(new Module_Assembly());
            }
        }

        public void Modify_Module(int nr)
        {

        }
        public void Modify_Frame(int moduleNr, int frameNr, Guid guid, Vector3 pos)
        {
            modules[moduleNr].Modify_Frame(frameNr, guid, pos);
        }
        public void Modify_Joint(int moduleNr, int jointNr, Guid guid)
        {
            modules[moduleNr].Modify_Joint(jointNr, guid);
        }
    }

    public class Module_Assembly
    {
        //Can create an agx assembly here, just to have it. 
        AgX_Frame[] frames;
        AgX_Joint joint;

        public Module_Assembly(/*all vars*/)//Adds frames&joint to the module
        {
            //frames[0] = new AgX_Frame(/*vars*/);
            //frames[1] = new AgX_Frame(/*vars*/);
            //joint = new AgX_Joint(/*vars*/);
        }

        public void Modify_Frame(int nr, Guid guid, Vector3 pos)//Modify a specific frame.
        {
            frames[nr].GetAgxObject().setPosition(Operations.ToAgxVec3(pos));
        }

        public void Modify_Joint(int nr, Guid guid) { }

    }

    public class AgX_Joint
    {
        private Guid guid;
        private string type;

        private agx.Constraint Joint;

        private agx.HingeFrame hinge_Frame = new agx.HingeFrame();//Might not need to be global

        public AgX_Joint(Guid guid)
        {
            this.guid = guid;
        }

        public void Create_Hinge(string type, Frame left, Frame right)
        {
            this.type = type;

            //Hinge is locked between the two objects.
            hinge_Frame.setCenter((left.frame.GetAgxObject().getPosition() + right.frame.GetAgxObject().getPosition()).Divide(2));
            if(left.rotation.z == 0)
                hinge_Frame.setAxis(new agx.Vec3(1, 0, 0)); //axis along the x direction
            else
                hinge_Frame.setAxis(new agx.Vec3(0, 1, 0)); //axis along the x direction
            Joint = new agx.Hinge(hinge_Frame,left.frame.GetAgxObject(),right.frame.GetAgxObject());
            //Joint.asHinge().getLock1D().setEnable(true);
            Joint.asHinge().getMotor1D().setEnable(true);
            Joint.asHinge().getRange1D().setEnable(true);
            //Might want to have this as a modifyable parameter:
            Joint.asHinge().getRange1D().setRange(-Mathf.PI/2, Mathf.PI/2);

            //Joint.asHinge().getMotor1D().setSpeed(0.2f);
        }
        public void Create_Lock(string type,Frame left, Frame right)
        {
            //connects right frame of left robot (LEFT) to left frame of right robot (RIGHT)
            Joint = new agx.LockJoint(left.frame.GetAgxObject(), right.frame.GetAgxObject(), (left.frame.GetAgxObject().getPosition() + right.frame.GetAgxObject().getPosition()).Divide(2));
        }

        public float Get_Angle()
        {
            return (float)Joint.asHinge().getAngle();
        }
        public void Set_Speed(float vel)
        {
            Joint.asHinge().getMotor1D().setSpeed(vel);
        }

        public void AddToSim()
        {
            Agx_Simulation.sim_Instance.add(Joint);
        }

    }

    public class AgX_Frame
    {
        private Guid guid;
        public string shape;
        private Vector3 size;
        private string materialName;
        private float restitution; private float friction;

        private agx.RigidBody agx_Object;
        
        public AgX_Frame(Guid guid, string shape, Vector3 size, Vector3 pos, Vector3 rot, float mass, bool isStatic, string materialName, float restitution, float friction)
        {
            this.guid = guid;

            this.shape = shape;
            this.size = size;
            ///Creates a geometry
            var dynamicRBGeometry = new agxCollide.Geometry();

            ///Creates the selected shape
            switch (shape)
            {
                case "Box": dynamicRBGeometry.add(new agxCollide.Box(Operations.ToAgxVec3(size))); break;
                case "Sphere": dynamicRBGeometry.add(new agxCollide.Sphere((size.x + size.y + size.z) / 3)); break;
            }

            agx_Object = new agx.RigidBody();
            ///Adds selected geometry to the rigidbody
            agx_Object.add(dynamicRBGeometry);

            agx_Object.setLocalPosition(Operations.ToAgxVec3(pos));///AgX

            agx_Object.setLocalRotation(new agx.EulerAngles(Operations.ToAgxVec3(rot)));///AgX

            agx_Object.getMassProperties().setMass(mass);

            agx_Object.getGeometries()[0].setMaterial(new agx.Material(materialName, restitution,friction));

            this.materialName = materialName;this.restitution = restitution;this.friction = friction;

            if (isStatic)
                agx_Object.setMotionControl(agx.RigidBody.MotionControl.STATIC);

            AddToSim();
        }

        /*--------------------------------------------------Object handling---------------------------------------------------*/
        /**-----------------------------------------------Return Algoryx object-----------------------------------------------*/
        public agx.RigidBody GetAgxObject()
        {
            return agx_Object;
        }

        public Guid Get_Guid()
        {
            return guid;
        }
        public string Get_Shape()
        {
            return shape;
        }
        public Vector3 Get_Position()
        {
            return Operations.FromAgxVec3(agx_Object.getLocalPosition());
        }
        public Vector3 Get_Size()
        {
            return size*2;//Size in unity is 2 times bigger.
        }
        public Vector3 Get_Rotation()
        {
            return Operations.FromAgxQuat(agx_Object.getLocalRotation()).eulerAngles;
        }
        public Quaternion Get_QuatRotation()
        {
            return Operations.FromAgxQuat(agx_Object.getLocalRotation());
        }
        public float Get_Mass()
        {
            return (float)agx_Object.getMassProperties().getMass();
        }

        /// Material:
        public string Get_MateriaName()
        {
            return materialName;
        }
        public float Get_Friction()
        {
            return friction;
        }
        public float Get_Restitution()
        {
            return restitution;
        }

        ///Simulation:
        public void AddToSim()
        {
            Agx_Simulation.sim_Instance.add(agx_Object);
        }

    }

    public static class Agx_Simulation
    {
        public static agxSDK.Simulation sim_Instance;

        public static void Start(float dt)
        {
            agx.agxSWIG.init();
            sim_Instance = new agxSDK.Simulation();//Initialize the simulation
            sim_Instance.setUniformGravity(new agx.Vec3(0, -9.80665f, 0));//set gravity in Y direction
            sim_Instance.getDynamicsSystem().getTimeGovernor().setTimeStep(dt);// set timestep
        }
        public static void StepForward()
        {
            sim_Instance.stepForward();
        }
        public static void Stop()
        {
            agx.agxSWIG.shutdown();
        }
    }
    
    public class Agx_Scene
    {

        Guid guid;
        agx.RigidBody terrain;


        /*--------------------------------------------------Creating terrain--------------------------------------------------*/
        //public void Create_Terrain(Guid guid, string heightmap, Vector3 position, string materialName, float restitution, float friction, float height)
        public Agx_Scene(Guid guid, List<Vector3> vertices, List<int> triangles,Vector3 position, string materialName, float restitution, float friction, float height)
        {
            this.guid = guid;

            //AgX:
            agx.Vec3Vector agx_vertices = new agx.Vec3Vector();
            agx.UInt32Vector agx_indices = new agx.UInt32Vector();
            for (int i = 0; i < vertices.Count; i++)
            {
                agx_vertices.Add(Operations.ToAgxVec3(vertices[i]));
            }
            for (int i = 0; i < triangles.Count; i++)
            {
                agx_indices.Add((uint)triangles[i]);
            }
            terrain = new agx.RigidBody();

            //uint optionsMask = (uint)agxCollide.Trimesh.TrimeshOptionsFlags.TERRAIN;
            var terrain_trimesh = new agxCollide.Trimesh(agx_vertices, agx_indices, "handmade terrain");//, optionsMask, height);

            var geometry = new agxCollide.Geometry();
            geometry.add(terrain_trimesh);
            geometry.setMaterial(new agx.Material(materialName,restitution,friction));
            
            terrain.add(geometry);
            terrain.setMotionControl(agx.RigidBody.MotionControl.STATIC);
            terrain.setLocalPosition(Operations.ToAgxVec3(position));//move right and -height for global 0
            
            ///Adds terrain to simulation
            //simulation.add(terrain);
            Agx_Simulation.sim_Instance.add(terrain);

        }
    }



    class Operations
    {
        /*-----------------------------------------------Mathematical operations----------------------------------------------*/
        /**----------------------------------------From agx.Vec3 to UnityEngine.Vector3---------------------------------------*/
        public static Vector3 FromAgxVec3(agx.Vec3 vec3)
        {
            return new Vector3((float)vec3.x, (float)vec3.y, (float)vec3.z);
        }
        /**----------------------------------------From UnityEngine.Vector3 to agx.Vec3---------------------------------------*/
        public static agx.Vec3 ToAgxVec3(Vector3 vector3)
        {
            return new agx.Vec3(vector3.x, vector3.y, vector3.z);
        }
        /**---------------------------------------From agx.Quat to UnityEngine.Quaternion-------------------------------------*/
        public static Quaternion FromAgxQuat(agx.Quat quat)//wrong
        {
            return new Quaternion((float)quat.x, (float)quat.y, (float)quat.z, (float)quat.w);
        }
    }
}

