using System.Collections;
using System.Collections.Generic;
using Simulation_Core;
using System;

public static class Robot_Optimization//IF we call the general class for Optimization, 
                               //then we can have inherited classes which specify this or parameters opti.
{
    public static bool started = false;
    public static float runspeed = 0.1f;
    public static int population = 3;

    static List<Opti_Dynamics> dynamics_List = new List<Opti_Dynamics>();

    static double[] originalGenome = new double[7] { 2 * (Math.PI / 9.0f), 0, Math.PI * 2.0f / 3.0f, 0, 4.0f,0,0};

    static string[] movementPattern = new string[] {"Left,Right,Forward"};

    public static void Load(Robot robot)//run this only once
    {

        //Create N dynamics scripts for this robot. (WHAT IS THE GENOME? maybe this variables)
        for(int i = 0; i<population; i++)
        {

            
            Opti_Dynamics dynamics;
            if (i == 0)
            {
                dynamics = new Opti_Dynamics(robot, originalGenome);
            }
            else
            {
                //DO SOMETHING GENOME STUFF WITH ORIGINAL GENOME random first time?
                var newGenome = ModifyGenome(originalGenome);
                dynamics = new Opti_Dynamics(robot, newGenome);
            }

            //Adds a new movement pattern to the list
            dynamics_List.Add(dynamics);
        }


    }

    public static bool Update(Robot robot, double time, int patternNumber, double endTime)
    {
        if (time < endTime)
        {
            AgX_Interface.Agx_Simulation.StepForward();

            //Update robot with dynamics[0] until time is reached. Then, check position/goal(fitness), and run next one

            if (time > 2)
                dynamics_List[patternNumber].Run(robot, time);

            robot.Update();
            return false;//not ended
        }
        else
        {
            //save result values
            //Create new robot from the serialization
            //return true (so that main can reset time)
            return true;
        }
    }

    public static void UpdatePopulation()//This one should mutate from the best survivors after time target is reached. 
    {

    }

    static double[] ModifyGenome(double[] original)
    {
        double[] newGenome = new double[7];
        newGenome = original;
        //cross or mutate?
        return newGenome;
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
