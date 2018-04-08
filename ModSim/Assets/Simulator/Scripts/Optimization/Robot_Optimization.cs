using System.Collections;
using System.Collections.Generic;
using Simulation_Core;
using System;
using System.Linq;

public static class Robot_Optimization//IF we call the general class for Optimization, 
                               //then we can have inherited classes which specify this or parameters opti.
{
    public static bool activated = false;
    public static bool started = false;
    public static float deltaTime = 0.01f;//physics sim time
    public static int population = 10;
    public static bool quickOpti = false;
    public static int IterTime = 30;

    public static int currentGeneration = 0;

    static List<Opti_Dynamics> dynamics_List = new List<Opti_Dynamics>();
                                                    //amplitudeP         AmpY    PhaseOffsetP        POY   Period OffsetP  OffsetY
    static double[] originalGenome = new double[7] { 2 * (Math.PI / 9.0f), 0, Math.PI * 2.0f / 3.0f, 0,    4.0f,    0,       0};
    static double[] UpperLimit = new double[7] { 4, 4, 8, 8, 10, 2, 2 };
    static double[] LowerLimit = new double[7] { -4, -4, -8, -8, 1, -2, -2 };

    static bool[] chosenForOptimization = new bool[7] { true, false, true, false, true, false, false };//at the moment

    static string[] movementPattern = new string[] {"Left,Right,Forward"};

    /*-----------------------Fitness:------------------------*/
    static AgX_Interface.Vector3 targetPosition = new AgX_Interface.Vector3(0,10,30);
    static int Xcompare = 1, Ycompare = 0, Zcompare = 1;
    

    public static void Load(Robot robot)//run this only once
    {
        //ASSIGN UPPER AND BOTTOM LIMIT
        //Create N dynamics scripts for this robot. (WHAT IS THE GENOME? maybe this variables)
        Random random = new Random();
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
                    if(chosenForOptimization[j])
                        newGenome[j] = GetRandomNumber(LowerLimit[j], UpperLimit[j], random);

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

    public static void Reset()
    {
        dynamics_List.Clear();
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

        return Math.Abs(Euclid_Distance);//Target is to get as close as possible to 0-vector
    }

    public static void UpdatePopulation(Robot robot)//This one should mutate from the best survivors after time target is reached. 
    {
        //Change to a genome (random) will be + or - (random), 0.1%-99% (random), to the specified value. 

        var newList = new List<Opti_Dynamics>();

        //Sort the list, ascending order, to get the best ones (smallest value) first
        dynamics_List = dynamics_List.OrderBy(o => o.fitnessValue).ToList();

        //CHECK
        for(int i = 0; i<dynamics_List.Count; i++)
            UnityEngine.Debug.Log(dynamics_List[i].fitnessValue);

        //Save length of dynamics list (could use population from r_opti, but this is a precaution for errors.
        int populationCount = dynamics_List.Count;

        if (dynamics_List.Count > 2)
        {
            newList.Add( dynamics_List[0]);
            newList.Add( dynamics_List[1]);
        }

        //Crossover 50% best solutions
        double percentile_50 = populationCount * 0.5;
        Random rnd = new Random();
        for (int i = 0; i < (int)percentile_50; i++)
        {
            if (newList.Count < populationCount)//ensure we don't make the list too big, in case user selects low population.
            {
                int r1 = rnd.Next((int)percentile_50);//index of best 50% population
                int r2 = rnd.Next((int)percentile_50);

                var newTmp = dynamics_List[r1];

                newTmp.genome = UniformCrossover(dynamics_List[r1].genome, dynamics_List[r2].genome);
                //add to new list, remove from old list.
                newList.Add(newTmp);//dynamics_List.RemoveAt(i);
            }
            
        }


        //Crossover rest from random existing solutions: 
        double percentile_70 = populationCount * 0.7;
        rnd = new Random();
        for(int i = 0; i< populationCount;i++)
        {
            if(newList.Count < populationCount)
            {
                int r1 = rnd.Next((int)percentile_70);//random of 70% best.
                int r2 = rnd.Next((int)percentile_70);//random of 70% best.

                var newTmp = dynamics_List[r1];

                newTmp.genome = UniformCrossover(dynamics_List[r1].genome, dynamics_List[r2].genome);

                newList.Add(newTmp);
            }
        }


        //Mutate function all population:
        //double random_20 = populationCount * 0.2;
        rnd = new Random();
        for (int i = 2; i<newList.Count;i++)//start from 2, want to save the two best.
        {
            newList[i].genome = Mutate(newList[i].genome,chosenForOptimization);//Mutates a random offspring
        }

        currentGeneration++;
        dynamics_List = newList;
        
    }

    private static double[] UniformCrossover(double[] first, double[] second)
    {
        //random selection
        Random rand = new Random();
        var result = new double[first.Length];
        //Choose children (uniformCrossover):
        result[0] = rand.Next(0, 2) == 0 ? first[0] : second[0];
        result[1] = rand.Next(0, 2) == 0 ? first[1] : second[1];
        result[2] = rand.Next(0, 2) == 0 ? first[2] : second[2];
        result[3] = rand.Next(0, 2) == 0 ? first[3] : second[3];
        result[4] = rand.Next(0, 2) == 0 ? first[4] : second[4];
        result[5] = rand.Next(0, 2) == 0 ? first[5] : second[5];
        result[6] = rand.Next(0, 2) == 0 ? first[6] : second[6];

        return result;
    }

    public static double GetRandomNumber(double minimum, double maximum, Random random)
    {
        return random.NextDouble() * (maximum - minimum) + minimum;
    }

    static double[] Mutate(double[] original, bool[] optimize)
    {
        double[] newGenome = new double[7];

        newGenome = original;

        //Random resetting:
        Random random = new Random();
        for(int i = 0; i< original.Length; i++)//each genome
        {
            if(optimize[i])//if current genome index is to be optimized
                if (random.Next(100) < 50)//35% prob
                    newGenome[i] = GetRandomNumber(LowerLimit[i], UpperLimit[i], random);
        }

        
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
