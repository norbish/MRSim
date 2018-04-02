using System.Collections;
using System.Collections.Generic;
using Simulation_Core;
using System;

public static class Robot_Optimization//IF we call the general class for Optimization, 
                               //then we can have inherited classes which specify this or parameters opti.
{
    public static bool started = false;
    public static float runspeed = 0.1f;
    static List<KeyValuePair<Robot, Opti_Dynamics>> robotsWithDynamics = new List<KeyValuePair<Robot, Opti_Dynamics>>();
    static double[] original = new double[] {1,2,3,4,5,6,7 };

    static string[] movementPattern = new string[] {"Left,Right,Forward"};

    public static List<Robot> Load(Robot original_Robot, float dt)//run this only once
    {
        runspeed = dt;
        //Create N robots like this robot. (WHAT IS THE GENOME? maybe this variables)
        //REMOVE THIS:
        int count = 0;
        for(int i = 0; i<2; i++)
        {
            Robot tmp_Robot = new Robot();
            tmp_Robot = original_Robot;

            //REMOVE THIS:
            foreach(Module mod in tmp_Robot.modules)
            {
                mod.frames[0].guid = mod.joint.leftFrameGuid = Guid.NewGuid();
                mod.frames[1].guid = mod.joint.rightFrameGuid = Guid.NewGuid();
               // mod.frames[0].position.x += count;
                //mod.frames[1].position.x += count;
                mod.joint.guid = Guid.NewGuid();
            }
            foreach(Sensor_Module mod in tmp_Robot.sensorModules)
            {
                mod.guid = Guid.NewGuid();
                //mod.position.x += count;
            }
            count+=3;

            
            tmp_Robot.Initialize();

            //DO SOMETHING GENOME STUFF WITH ORIGINAL GENOME random first time?

            Opti_Dynamics dynamics = new Opti_Dynamics(tmp_Robot, original);

            robotsWithDynamics.Add(new KeyValuePair<Robot, Opti_Dynamics>(tmp_Robot,dynamics));
        }
        List<Robot> robots = new List<Robot>();
        foreach (KeyValuePair<Robot, Opti_Dynamics> rwd in robotsWithDynamics)
        {
            robots.Add(rwd.Key);
        }

        return robots;
    }

    public static void Update(double time)
    {
        AgX_Interface.Agx_Simulation.StepForward();

        foreach(KeyValuePair<Robot,Opti_Dynamics> rwd in robotsWithDynamics)
        {
            if(time > 2)
            rwd.Value.Run(rwd.Key, time);

            rwd.Key.Update();
        }
    }

    static void UpdatePopulation()//This one should mutate from the best survivors after time target is reached. 
    {

    }

    static Robot GetRobots()
    {
        return new Robot();
    }

    public class Opti_Dynamics
    {
        public double[] angles;
        public double[] amplitudes;
        public double[] period;
        public double[] phaseDiff;
        public double[] offset;

        public double[] genome = new double[7];

        static double PitchMoveDirection = 1;static double YawMoveDirection = 1;
        static double PitchTurnDirection = 1;static double YawTurnDirection = 1;
        static double set_period = 4.0f;
        static bool NewAction = false;

        static double angle; //rads

        public Opti_Dynamics(Robot robot, double[] genome)
        {
            //Set robot parameters (GENOME)
            Initialize(robot, genome[0],genome[1], PitchMoveDirection * genome[2], YawMoveDirection * genome[3], genome[4], PitchMoveDirection * genome[5], YawTurnDirection * genome[6]);
            this.genome = genome;
            
        }

        public void Run(Robot robot, double t)
        {
            //Set angles
            Opti_angles(robot, t);
        }

        public void Initialize(Robot robot, double _ampPitch, double _ampYaw, double _phaseOffsetPitch, double _phaseOffsetYaw, double _period, double _offsetPitch, double _offsetYaw)
        {
            int mod_n = robot.modules.Count;
            amplitudes = new double[mod_n];
            period = new double[mod_n];
            phaseDiff = new double[mod_n];
            offset = new double[mod_n];

            //can set "buttons" for each turning, rolling, etc.
            for (int i = 0; i < robot.modules.Count; i++)
            {
                period[i] = _period;

                if (robot.modules[i].Axis == "Pitch")
                {
                    amplitudes[i] = _ampPitch;
                    phaseDiff[i] = 0;// _phaseDiffPitch;
                    offset[i] = _offsetPitch;
                }

                else
                {
                    amplitudes[i] = _ampYaw;
                    phaseDiff[i] = 0;// _phaseDiffYaw;
                    offset[i] = _offsetYaw;
                }


                for (int j = 0; j < mod_n; j++)//Phase offset
                {
                    if (j > 1)//Not taking into account the first pitch and first yaw which should be 0
                    {
                        if (robot.modules[j].Axis == "Pitch")
                            phaseDiff[j] = phaseDiff[j - 2] + _phaseOffsetPitch;
                        else
                            phaseDiff[j] = phaseDiff[j - 2] + _phaseOffsetYaw;
                    }
                }
            }
        }
        
       

        public static void Reset(Robot robot, double t)
        {
            foreach (var mod in robot.modules)
            {
                mod.joint.Reset_Angle();
            }
        }

        //this:
        public void Opti_angles(Robot robot, double t)//movement
        {

            for (int i = 0; i < robot.modules.Count; i++)
            {
                if (robot.modules[i].Axis == "Pitch")
                {

                    angle = amplitudes[i] * Math.Sin(2 * Math.PI * t / period[i] + phaseDiff[i]) + offset[i]; //Angle = amplitude + sin(2pi * t / period + phase diff) + offset
                    robot.modules[i].joint.MOVE(angle);
                }
                //Reduntant, dont need the IFs
                if (robot.modules[i].Axis == "Yaw")
                {

                    angle = amplitudes[i] * Math.Sin(2 * Math.PI * t / period[i] + phaseDiff[i]) + offset[i]; //Angle = amplitude + sin(2pi * t / period + phase diff) + offset
                    robot.modules[i].joint.MOVE(angle);
                }
            }
        }

    }
}
