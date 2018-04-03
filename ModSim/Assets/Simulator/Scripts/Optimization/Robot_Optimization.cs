using System.Collections;
using System.Collections.Generic;
using Simulation_Core;
using System;
using System.Linq;

public static class Robot_Optimization//IF we call the general class for Optimization, 
                               //then we can have inherited classes which specify this or parameters opti.
{
    public static bool started = false;
    public static float runspeed = 0.1f;
    public static int population = 3;

    static List<Opti_Dynamics> dynamics_List = new List<Opti_Dynamics>();
                                                    //amplitudeP         AmpY    PhaseOffsetP        POY   Period OffsetP  OffsetY
    static double[] originalGenome = new double[7] { 2 * (Math.PI / 9.0f), 0, Math.PI * 2.0f / 3.0f, 0,    4.0f,    0,       0};
    static double[] UpperLimit = new double[7] { 2, 2, 8, 8, 12, 2, 2 };
    static double[] LowerLimit = new double[7] { -2, -2, -8, -8, -12, -2, -2 };

    static string[] movementPattern = new string[] {"Left,Right,Forward"};

    /*-----------------------Fitness:------------------------*/
    static AgX_Interface.Vector3 targetPosition = new AgX_Interface.Vector3(0,10,10);
    static int Xcompare = 1, Ycompare = 0, Zcompare = 1;
    

    public static void Load(Robot robot)//run this only once
    {
        //ASSIGN UPPER AND BOTTOM LIMIT
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
                double[] newGenome = new Double[7];
                for (int j = 0; j < newGenome.Length; j++)//populate random genome
                    newGenome[j] = GetRandomNumber(LowerLimit[j], UpperLimit[j]);

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
            //Calculate the fitness of current solution
            dynamics_List[patternNumber].fitnessValue = CalculateFitness(robot);
            //save result values
            //Create new dynamics from the serialization
            //return true (so that main can reset time)
            return true;
        }
    }
    static double CalculateFitness(Robot robot)
    {
        //get robot position
        var R = robot.position;
        var T = targetPosition;

        var x = Math.Pow((T.x - R.x), 2) * Xcompare;
        var y = Math.Pow((T.y - R.y), 2) * Ycompare;
        var z = Math.Pow((T.z - R.z), 2) * Zcompare;

        var Euclid_Distance = Math.Sqrt( x + y + z );

        return Euclid_Distance;
    }

    public static void UpdatePopulation(Robot robot)//This one should mutate from the best survivors after time target is reached. 
    {
        //Select the 40% best ones and 30% random solutions, and 30% new "random" ones (meaning set inbetween certain values).
        //Change to a genome (random) will be + or - (random), 0.1%-99% (random), to the specified value. 

        var newList = new List<Opti_Dynamics>();

        dynamics_List.OrderBy(o => o.genome).ToList();

        foreach (Opti_Dynamics y in dynamics_List)
            UnityEngine.Debug.Log(y.fitnessValue);

        int listCount = dynamics_List.Count;

        //Get 40 best solutions
        double percentile_40 = listCount * 0.4;

        for (int i = 0; i < (int)percentile_40; i++)
        {
            newList.Add(dynamics_List[i]);dynamics_List.RemoveAt(i);
        }

        //get 30% random solutions:
        double randomSelection_30 = listCount * 0.3;

        for(int i = 0; i< (int)randomSelection_30;i++)
        {
            if(newList.Count < Robot_Optimization.population)
            {
                Random rnd = new Random();
                int r = rnd.Next(dynamics_List.Count);//random with max as list count.

                newList.Add(dynamics_List[r]);dynamics_List.RemoveAt(r);
            }
        }

        //Get 30% breeding genomes (the rest):
        

        //Change to a genome (random) will be + or - (random), 0.1%-99% (random), to the specified value. 

       
    }

    private static void Breeding()
    {
        
    }

    public static double GetRandomNumber(double minimum, double maximum)
    {
        Random random = new Random();
        return random.NextDouble() * (maximum - minimum) + minimum;
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
        private double[] angles;
        private double[] amplitudes;
        private double[] period;
        private double[] phaseDiff;
        private double[] offset;

        public double[] genome = new double[7];
        public double fitnessValue = 0;

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

        private void Initialize(Robot robot, double _ampPitch, double _ampYaw, double _phaseOffsetPitch, double _phaseOffsetYaw, double _period, double _offsetPitch, double _offsetYaw)
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
