/*Algoryx physics Interface class (AgX_Interface)
 * Torstein Sundnes Lenerand
 * NTNU Ålesund
 */

 ///This class contains the AgX object. 

using System;
using System.Collections.Generic;
using System.Linq;
//using Simulation_Core;

namespace AgX_Interface
{
    public static class AgX_Assembly
    {
        static agxSDK.Assembly robotAssembly = new agxSDK.Assembly();


        public static void AddToAssembly(agx.RigidBody body)
        {
            robotAssembly.add(body);
        }
        public static void AddToAssembly(agx.Constraint joint)
        {
            robotAssembly.add(joint);
        }

        public static Vector3 GetPosition()
        {
            return Operations.FromAgxVec3(robotAssembly.getPosition());
            //return Operations.FromAgxVec3(robotAssembly.getFrame().getTranslate());
        }
        public static Quaternion GetRotation()
        {
            return Operations.FromAgxQuat(robotAssembly.getRotation());
        }

        public static void SetPosition(Vector3 pos)
        {
            robotAssembly.setPosition(Operations.ToAgxVec3(pos));
        }
        public static void SetRotation(Quaternion rot)
        {
            robotAssembly.setRotation(Operations.ToAgxQuat(rot));
        }

        public static void AddToSim()
        {
            Agx_Simulation.sim_Instance.add(robotAssembly, true);
        }
        public static void RemoveFromSim()
        {
            Agx_Simulation.sim_Instance.remove(robotAssembly, true);
            robotAssembly = new agxSDK.Assembly();
        }
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

        public void Create_Hinge(string type, AgX_Frame left, AgX_Frame right, double leftLimit, double rightLimit)
        {
            this.type = type;

            //Hinge is locked between the two objects.
            hinge_Frame.setCenter((left.GetAgxObject().getPosition() + right.GetAgxObject().getPosition()).Divide(2));
            if (left.Get_Rotation().x == 0)//pitch or yaw module
            {
                hinge_Frame.setAxis(new agx.Vec3(1, 0, 0)); //axis along the x direction
                //UnityEngine.Debug.Log("z in agxint: " + left.Get_Rotation().x);
            }
            else
                hinge_Frame.setAxis(new agx.Vec3(0, 1, 0)); //axis along the x direction

            Joint = new agx.Hinge(hinge_Frame, left.GetAgxObject(), right.GetAgxObject());
            //Joint.asHinge().getLock1D().setEnable(true);
            Joint.asHinge().getMotor1D().setEnable(true);
            Joint.asHinge().getRange1D().setEnable(true);
            //Might want to have this as a modifyable parameter:
            Joint.asHinge().getRange1D().setRange(leftLimit, rightLimit/*-Math.PI / 2, Math.PI / 2*/);

            //Joint.asHinge().getMotor1D().setSpeed(0.2f);
        }
        public void Create_Lock(string type, AgX_Frame left, AgX_Frame right)
        {
            //connects right frame of left robot (LEFT) to left frame of right robot (RIGHT)
            Joint = new agx.LockJoint(left.GetAgxObject(), right.GetAgxObject(), (left.GetAgxObject().getPosition() + right.GetAgxObject().getPosition()).Divide(2));
        }

        //Sensory module locks:
        public void Create_Lock(string type, AgX_Frame right, AgX_Primitive s_mod)
        {
            //Creates a joint with a specified middle position for the lockframe.
            //THIS IS NOT THE MIDDLE OF THE LOCK FRAME (frames are longer than sensors)
            Joint = new agx.LockJoint(right.GetAgxObject(), s_mod.GetAgxObject(), (right.GetAgxObject().getPosition() + s_mod.GetAgxObject().getPosition()).Divide(2));
        }
        public void Create_Lock(string type, AgX_Primitive s_mod, AgX_Frame left)
        {
            //THIS IS NOT THE MIDDLE OF THE LOCK FRAME (frames are longer than sensors)
            Joint = new agx.LockJoint(s_mod.GetAgxObject(), left.GetAgxObject(), (left.GetAgxObject().getPosition() + s_mod.GetAgxObject().getPosition()).Divide(2));
        }

        public void ForceSensorLock(AgX_Primitive sm, AgX_ForceSensor fs, Vector3 lockPosition)
        {
            Joint = new agx.LockJoint(sm.GetAgxObject(), fs.GetAgxObject(), Operations.ToAgxVec3(lockPosition));
            Joint.setEnableComputeForces(true);
        }
        public void DistanceSensorLock(AgX_Primitive sm, AgX_Primitive ds, Vector3 lockPosition)
        {
            Joint = new agx.LockJoint(sm.GetAgxObject(), ds.GetAgxObject(), Operations.ToAgxVec3(lockPosition));
            Joint.setEnableComputeForces(true);
        }
        public double GetForce()
        {
            return Joint.getCurrentForce(0);
        }
        public double Get_Angle()
        {
            return Joint.asHinge().getAngle();
        }
        /*public Vector3 GetJointForce(AgX_ForceSensor fs)
        {
            agx.Vec3 force = new agx.Vec3();
            agx.Vec3 torque = new agx.Vec3();
            Joint.getLastForce(fs.GetAgxObject(), force, torque);
            return Operations.FromAgxVec3(force);
        }*/
        public void Set_Speed(double vel)
        {
            Joint.asHinge().getMotor1D().setSpeed(vel);
        }

        public void AddToSim()
        {
            AgX_Assembly.AddToAssembly(Joint);
            //Agx_Simulation.sim_Instance.add(Joint);
        }
        public void Remove()
        {
            Agx_Simulation.sim_Instance.remove(Joint);
        }

    }

    public class AgX_ForceSensor
    {
        private Guid guid;
        private Vector3 scale;
        private agx.RigidBody agx_Object;

        public AgX_ForceSensor(Guid guid, string materialName, Vector3 pos, Quaternion rot, Vector3 scale, double mass)
        {
            this.guid = guid;
            this.scale = scale;

            var dynamicRBGeometry = new agxCollide.Geometry();///AgX

            dynamicRBGeometry.add(new agxCollide.Box(Operations.ToAgxVec3(scale)));

            dynamicRBGeometry.setMaterial(new agx.Material(materialName));

            agx_Object = new agx.RigidBody();

            agx_Object.add(dynamicRBGeometry);

            agx_Object.setPosition(Operations.ToAgxVec3(pos));

            agx_Object.setRotation(Operations.ToAgxQuat(rot));

            agx_Object.getMassProperties().setMass(mass);

            Agx_Simulation.sim_Instance.add(agx_Object);
        }

       /* public Vector3 GetForce()
        {
            return Operations.FromAgxVec3(agx_Object.getForce());
        }*/

        public agx.RigidBody GetAgxObject()
        {
            return agx_Object;
        }
        public Vector3 GetPosition()
        {
            return Operations.FromAgxVec3(agx_Object.getPosition());
        }
        public Quaternion GetRotation()
        {
            return Operations.FromAgxQuat(agx_Object.getRotation());
        }
        public void AddToSim()
        {
            AgX_Assembly.AddToAssembly(agx_Object);
        }
        public void Remove()
        {
            Agx_Simulation.sim_Instance.remove(agx_Object);
        }
    }

    public class AgX_Primitive
    {
        private Guid guid;
        private string shape;
        private Vector3 size;
        private string materialName;

        private agx.RigidBody agx_Object;

        public AgX_Primitive(Guid guid, string shape, Vector3 pos, Quaternion rot, Vector3 size, double mass, string materialName, bool isStatic, bool AddToRobot)
        {
            this.guid = guid;
            this.shape = shape;
            this.size = size;
            this.materialName = materialName;

            var dynamicRBGeometry = new agxCollide.Geometry();

            switch (this.shape)
            {
                case "Box": dynamicRBGeometry.add(new agxCollide.Box(Operations.ToAgxVec3(this.size))); break;
                case "Sphere": dynamicRBGeometry.add(new agxCollide.Sphere((this.size.x + this.size.y + this.size.z) / 3)); break;
                default: dynamicRBGeometry.add(new agxCollide.Box(Operations.ToAgxVec3(this.size))); break; 
            }

            dynamicRBGeometry.setMaterial(new agx.Material(this.materialName));

            agx_Object = new agx.RigidBody();
            agx_Object.add(dynamicRBGeometry);
            agx_Object.setLocalPosition(Operations.ToAgxVec3(pos));///AgX

            agx_Object.setLocalRotation(new agx.Quat(rot.x, rot.y, rot.z, rot.w));///AgX

            agx_Object.getMassProperties().setMass(mass);

            if (isStatic)
                agx_Object.setMotionControl(agx.RigidBody.MotionControl.STATIC);

            if (AddToRobot)
                AddToSim();
            else
                Agx_Simulation.sim_Instance.add(agx_Object);
        }

        public Vector3 Get_Position()
        {
            return Operations.FromAgxVec3(agx_Object.getPosition());
        }
        /*public Vector3 Get_Rotation()
        {
            return Operations.FromAgxQuat(agx_Object.GetRotation()).ToEulerRad();
        }*/
        public Quaternion Get_QuatRotation()
        {
            return Operations.FromAgxQuat(agx_Object.getRotation());
        }
        public void AddToSim()
        {
            AgX_Assembly.AddToAssembly(agx_Object);
            //Agx_Simulation.sim_Instance.add(agx_Object);
        }
        public agx.RigidBody GetAgxObject()
        {
            return agx_Object;
        }
        public void Remove()
        {
            Agx_Simulation.sim_Instance.remove(agx_Object);
        }
    }


    public class AgX_Frame
    {
        private Guid guid;
        public string shape;
        private double size;
        private string materialName;

        private agx.RigidBody agx_Object;

        public AgX_Frame(Guid guid, string shape, Vector3[] vertices, Vector2[] uvs, int[] triangles, double size, Vector3 pos, Quaternion rot, double mass, bool isStatic, string materialName)
        {
            this.guid = guid;

            this.shape = shape;
            this.size = size;
            this.materialName = materialName;

            //scale by 2, to fit unity.
            /* Vector3[] tmp_verts = vertices;
             for (int i = 0; i < tmp_verts.Length; i++)
             {
                 tmp_verts[i].x *= 2;
                 tmp_verts[i].y *= 2;
                 tmp_verts[i].z *= 2;
             }*/

            var tri = new agxCollide.Trimesh(Operations.ToAgxVec3Vector(vertices), Operations.ToAgxIntVector(triangles), "stdFrame");
            ///Creates a geometry
            var dynamicRBGeometry = new agxCollide.Geometry();
            dynamicRBGeometry.add(tri);

            dynamicRBGeometry.setMaterial(new agx.Material(materialName));

            ///Creates the selected shape
            /*switch (shape)
            {
                case "Box": dynamicRBGeometry.add(new agxCollide.Box(Operations.ToAgxVec3(size))); break;
                case "Sphere": dynamicRBGeometry.add(new agxCollide.Sphere((size.x + size.y + size.z) / 3)); break;
            }*/

            agx_Object = new agx.RigidBody();
            ///Adds selected geometry to the rigidbody
            agx_Object.add(dynamicRBGeometry);

            agx_Object.setLocalPosition(Operations.ToAgxVec3(pos));///AgX

            //var y = new agx.EulerAngles(Operations.ToAgxVec3(rot));

            //UnityEngine.Debug.Log("x: " + y.x + ", y: " + y.y + ", z: " + y.z);

            agx_Object.setLocalRotation(new agx.Quat(rot.x, rot.y, rot.z, rot.w));///AgX

            //agx_Object.setLocalRotation(new agx.EulerAngles(Operations.ToAgxVec3(rot)));///AgX

            //UnityEngine.Debug.Log("x: " +agx_Object.GetPosition().x + ", y: " + agx_Object.GetPosition().y + ", z: " + agx_Object.GetPosition().z);

            agx_Object.getMassProperties().setMass(mass);

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
            return Operations.FromAgxVec3(agx_Object.getPosition());
        }
        public double Get_Size()
        {
            return size * 2;//Size in unity is 2 times bigger.
        }
        public agx.Vec3 Get_Rotation()
        {
            //UnityEngine.Debug.Log(agx_Object.GetRotation().asVec3().x+","+agx_Object.GetRotation().asVec3().y+","+agx_Object.GetRotation().asVec3().z);
            return agx_Object.getRotation().asVec3();
        }
        public Quaternion Get_QuatRotation()
        {
            return Operations.FromAgxQuat(agx_Object.getRotation());
        }
        public double Get_Mass()
        {
            return (double)agx_Object.getMassProperties().getMass();
        }

        /// Material:
        public string Get_MateriaName()
        {
            return materialName;
        }

        ///Simulation:
        public void AddToSim()
        {
            AgX_Assembly.AddToAssembly(agx_Object);
            //Agx_Simulation.sim_Instance.add(agx_Object);
        }
        public void Remove()
        {
            Agx_Simulation.sim_Instance.remove(agx_Object);
        }

    }

    public static class Agx_Simulation
    {
        public static agxSDK.Simulation sim_Instance;

        public static void Start(double dt)
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
            RemoveSimObjects();
            sim_Instance = null;
            agx.agxSWIG.shutdown();

        }
        public static void RemoveSimObjects()
        {
            if (sim_Instance != null)
            {
                sim_Instance.removeAllObjects();
            }
            sim_Instance = null;
        }
        public static void AddContactMaterial(string a, string b, double restitution, double friction, double youngsModulus)
        {
            var material_A = new agx.Material(a);
            var material_B = new agx.Material(b);
            var contact_material = new agx.ContactMaterial(material_A, material_B);
            contact_material.setFrictionCoefficient(restitution);
            contact_material.setRestitution(friction);
            contact_material.setYoungsModulus(youngsModulus);
            sim_Instance.add(material_A);
            sim_Instance.add(material_B);
            sim_Instance.add(contact_material);
        }
        public static void Reset()
        {

        }
    }

    public class Agx_Scene
    {

        Guid guid;
        agx.RigidBody terrain;


        /*--------------------------------------------------Creating terrain--------------------------------------------------*/
        //public void Create_Terrain(Guid guid, string heightmap, Vector3 position, string materialName, double restitution, double friction, double height)
        public Agx_Scene(Guid guid, List<Vector3> vertices, List<int> triangles, Vector3 position, string materialName, double height)
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
            geometry.setMaterial(new agx.Material(materialName));

            terrain.add(geometry);
            terrain.setMotionControl(agx.RigidBody.MotionControl.STATIC);

            //position.y -= height;
            terrain.setLocalPosition(Operations.ToAgxVec3(position));//move right and -height for global 0

            ///Adds terrain to simulation
            //simulation.add(terrain);
            Agx_Simulation.sim_Instance.add(terrain);

        }
        public void Remove()
        {
            Agx_Simulation.sim_Instance.remove(terrain);
        }
    }



    class Operations
    {
        /*-----------------------------------------------Mathematical operations----------------------------------------------*/
        /**----------------------------------------From agx.Vec3 to UnityEngine.Vector3---------------------------------------*/
        public static Vector3 FromAgxVec3(agx.Vec3 vec3)
        {
            return new Vector3((double)vec3.x, (double)vec3.y, (double)vec3.z);
        }
        /**----------------------------------------From UnityEngine.Vector3 to agx.Vec3---------------------------------------*/
        public static agx.Vec3 ToAgxVec3(Vector3 vector3)
        {
            return new agx.Vec3(vector3.x, vector3.y, vector3.z);
        }
        /**---------------------------------------From agx.Quat to UnityEngine.Quaternion-------------------------------------*/
        public static Quaternion FromAgxQuat(agx.Quat quat)
        {
            return new Quaternion((double)quat.x, (double)quat.y, (double)quat.z, (double)quat.w);
        }
        public static agx.Quat ToAgxQuat(Quaternion quat)
        {
            return new agx.Quat(quat.x, quat.y, quat.z, quat.w);
        }

        public static agx.Vec3Vector ToAgxVec3Vector(Vector3[] vector3)
        {
            agx.Vec3Vector vec3 = vector3.Count() > 0 ? new agx.Vec3Vector(vector3.Count()) : new agx.Vec3Vector();

            for (int i = 0; i < vector3.Count(); i++)
            {
                vec3.Add(new agx.Vec3(vector3[i].x, vector3[i].y, vector3[i].z));
            }
            return vec3;
        }
        public static agx.UInt32Vector ToAgxIntVector(int[] integers)
        {
            agx.UInt32Vector intVec = integers.Count() > 0 ? new agx.UInt32Vector(integers.Count()) : new agx.UInt32Vector();

            for (int i = 0; i < integers.Count(); i++)
            {
                intVec.Add((uint)integers[i]);
            }
            return intVec;
        }
    }

    /*-------------------------------------------------Utility Functions:-------------------------------------------------*/
    /*------------------------------------------------------Vector3-------------------------------------------------------*/

    public struct Vector3
    {
        public double x, y, z;

        const double Rad2Deg = (180 / Math.PI);
        const double Deg2Rad = (Math.PI / 180);

        public static Vector3 forward = new Vector3(0, 0, 1);
        public static Vector3 zero = new Vector3(0f, 0f, 0f);

        public Vector3(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public static double Length(Vector3 a)
        {
            return Math.Sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        }

        public static Vector3 Normalize(Vector3 a)
        {
            double length = Length(a);
            if (length != 0)
            {
                a.x = a.x / length;
                a.y = a.y / length;
                a.z = a.z / length;
            }
            return a;
        }

        public Vector3 toRad()
        {
            var v3 = new Vector3();

            v3.x = x * Deg2Rad;
            v3.y = y * Deg2Rad;
            v3.z = z * Deg2Rad;

            return v3;
        }

        public static Vector3 Lerp(Vector3 a, Vector3 b, double amount)
        {
            return amount * Vector3.Normalize(b - a) + a;
        }

        //Operators:
        public static Vector3 operator +(Vector3 a, Vector3 b)
        {
            a.x += b.x;
            a.y += b.y;
            a.z += b.z;
            return a;
        }
        public static Vector3 operator -(Vector3 a, Vector3 b)
        {
            a.x -= b.x;
            a.y -= b.y;
            a.z -= b.z;
            return a;
        }
        public static Vector3 operator *(Vector3 a, double scale)
        {
            a.x *= scale;
            a.y *= scale;
            a.z *= scale;
            return a;
        }
        public static Vector3 operator *(double scale, Vector3 a)
        {
            a.x *= scale;
            a.y *= scale;
            a.z *= scale;
            return a;
        }
        public static Vector3 operator /(Vector3 a, double scale)
        {
            a.x /= scale;
            a.y /= scale;
            a.z /= scale;
            return a;
        }


    }

    public struct Vector2
    {
        public double x, y;

        public Vector2(double x, double y)
        {
            this.x = x;
            this.y = y;
        }
    }

    public struct Quaternion
    {
        const double Rad2Deg = (180 / Math.PI);
        const double Deg2Rad = (Math.PI / 180);
        public double x, y, z, w;

        static Quaternion identity = new Quaternion(0, 0, 0, 1);

        public Quaternion(double x, double y, double z, double w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }


        public Vector3 ToEulerRad()
        {
            double sqw = this.w * this.w;
            double sqx = this.x * this.x;
            double sqy = this.y * this.y;
            double sqz = this.z * this.z;
            double unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
            double test = this.x * this.w - this.y * this.z;
            Vector3 v = new Vector3();

            if (test > 0.4995f * unit)
            { // singularity at north pole
                v.y = 2f * Math.Atan2(this.y, this.x);
                v.x = Math.PI / 2;
                v.z = 0;
                return NormalizeAngles(v * Rad2Deg);
            }
            if (test < -0.4995f * unit)
            { // singularity at south pole
                v.y = -2f * Math.Atan2(this.y, this.x);
                v.x = -Math.PI / 2;
                v.z = 0;
                return NormalizeAngles(v * Rad2Deg);
            }
            Quaternion q = new Quaternion(this.w, this.z, this.x, this.y);
            v.y = System.Math.Atan2(2f * q.x * q.w + 2f * q.y * q.z, 1 - 2f * (q.z * q.z + q.w * q.w));     // Yaw
            v.x = System.Math.Asin(2f * (q.x * q.z - q.w * q.y));                             // Pitch
            v.z = System.Math.Atan2(2f * q.x * q.y + 2f * q.z * q.w, 1 - 2f * (q.y * q.y + q.z * q.z));      // Roll
            return NormalizeAngles(v * (180 / Math.PI));
        }
        private static Vector3 NormalizeAngles(Vector3 angles)
        {
            angles.x = NormalizeAngle(angles.x);
            angles.y = NormalizeAngle(angles.y);
            angles.z = NormalizeAngle(angles.z);
            return angles;
        }
        private static double NormalizeAngle(double angle)
        {
            while (angle > 360)
                angle -= 360;
            while (angle < 0)
                angle += 360;
            return angle;
        }

        public static Quaternion FromEulerRad(Vector3 euler)
        {
            var yaw = euler.x;
            var pitch = euler.y;
            var roll = euler.z;
            double rollOver2 = roll * 0.5;
            double sinRollOver2 = System.Math.Sin(rollOver2);
            double cosRollOver2 = System.Math.Cos(rollOver2);
            double pitchOver2 = pitch * 0.5f;
            double sinPitchOver2 = System.Math.Sin(pitchOver2);
            double cosPitchOver2 = System.Math.Cos(pitchOver2);
            double yawOver2 = yaw * 0.5f;
            double sinYawOver2 = System.Math.Sin(yawOver2);
            double cosYawOver2 = System.Math.Cos(yawOver2);
            Quaternion result;
            result.x = sinYawOver2 * cosPitchOver2 * cosRollOver2 + cosYawOver2 * sinPitchOver2 * sinRollOver2; // confirmed (scc+css)
            result.y = cosYawOver2 * sinPitchOver2 * cosRollOver2 - sinYawOver2 * cosPitchOver2 * sinRollOver2; // confirmed (csc-scs)
            result.z = cosYawOver2 * cosPitchOver2 * sinRollOver2 - sinYawOver2 * sinPitchOver2 * cosRollOver2; // confirmed (ccs-ssc)
            result.w = cosYawOver2 * cosPitchOver2 * cosRollOver2 + sinYawOver2 * sinPitchOver2 * sinRollOver2; // confirmed (ccc+sss)
            return result;
        }

        public Vector3 eulerAnglesD()
        {
            Vector3 vector;

            double unit = x * x + y * y + z * z + w * w;
            double test = x * y + z * w;

            if (test > 0.4999f * unit)                              // 0.4999f OR 0.5f - EPSILON
            {
                // Singularity at north pole
                vector.y = 2f * (double)Math.Atan2(x, w);  // Yaw
                vector.x = Math.PI * 0.5f;                         // Pitch
                vector.z = 0f;                                // Roll
                return vector;
            }
            else if (test < -0.4999f * unit)                        // -0.4999f OR -0.5f + EPSILON
            {
                // Singularity at south pole
                vector.y = -2f * (double)Math.Atan2(x, w); // Yaw
                vector.x = -Math.PI * 0.5f;                        // Pitch
                vector.z = 0f;                                // Roll
                return vector;
            }
            else
            {

                vector.y = (double)Math.Atan2(2f * x * w + 2f * y * z, 1 - 2f * (z * z + w * w));// * Math.PI/180;     // Yaw to degrees
                vector.x = (double)Math.Asin(2f * (x * z - w * y));// * Math.PI / 180; ;                             // Pitch 
                vector.z = (double)Math.Atan2(2f * x * y + 2f * z * w, 1 - 2f * (y * y + z * z));// * Math.PI / 180; ;      // Roll 

                return vector;
            }
        }

        public static Vector3 operator *(Quaternion quat, Vector3 vec)
        {
            double num = quat.x * 2f;
            double num2 = quat.y * 2f;
            double num3 = quat.z * 2f;
            double num4 = quat.x * num;
            double num5 = quat.y * num2;
            double num6 = quat.z * num3;
            double num7 = quat.x * num2;
            double num8 = quat.x * num3;
            double num9 = quat.y * num3;
            double num10 = quat.w * num;
            double num11 = quat.w * num2;
            double num12 = quat.w * num3;
            Vector3 result;
            result.x = (1f - (num5 + num6)) * vec.x + (num7 - num12) * vec.y + (num8 + num11) * vec.z;
            result.y = (num7 + num12) * vec.x + (1f - (num4 + num6)) * vec.y + (num9 - num10) * vec.z;
            result.z = (num8 - num11) * vec.x + (num9 + num10) * vec.y + (1f - (num4 + num5)) * vec.z;
            return result;
        }

    }

}

