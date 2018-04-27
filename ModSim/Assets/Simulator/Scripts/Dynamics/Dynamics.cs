/* Dynamics, robot movement class (Dynamics)
 * Torstein Sundnes Lenerand
 * NTNU Ålesund
 */

using System.Collections;
using System.Collections.Generic;
using System;
//using UnityEngine;

public static class Dynamics
{
    public static int mode = 1; //Must be changed if the script is customized. 
    public static string currentAction = "Idle";//check this string to see if have to initialize
    public static string action = "Idle";// = "Turn";
    public static double[] angles;
    public static double[] amplitudes;
    public static double[] period;
    public static double[] phaseDiff;
    public static double[] offset;

    public static double[] f_movementVars = new double[7] { 2 * (double)(Math.PI / 9.0), 0, (double)Math.PI * 2.0 / 3.0f, 0, 4.0f, 0, 0 };
    public static double[] t_movementVars = new double[7] { 2 * (double)(Math.PI / 9.0), 0, (double)Math.PI * 2.0 / 3.0f, 0, 4.0f, 0, 20 * (double)Math.PI / 180 };

    static double angle; //rads

    public static void Initialize(string newAction, Simulation_Core.Robot robot, double _ampPitch,double _ampYaw, double _phaseOffsetPitch, double _phaseOffsetYaw, double _period, double _offsetPitch, double _offsetYaw)
    {
        Dynamics.action = newAction;
        int mod_n = robot.modules.Count;
        amplitudes = new double[mod_n];
        period = new double[mod_n];
        phaseDiff = new double[mod_n];
        offset = new double[mod_n];

        //can set "buttons" for each turning, rolling, etc.
        for (int i = 0; i < robot.modules.Count; i++)
        {
            Dynamics.period[i] = _period;

            if (robot.modules[i].axis == "Pitch")
            {
                Dynamics.amplitudes[i] = _ampPitch;
                Dynamics.phaseDiff[i] = 0;// _phaseDiffPitch;
                Dynamics.offset[i] = _offsetPitch;
            }

            else
            {
                Dynamics.amplitudes[i] = _ampYaw;
                Dynamics.phaseDiff[i] = 0;// _phaseDiffYaw;
                Dynamics.offset[i] = _offsetYaw;
            }
        }

        switch (newAction)
        {
            case "Turn":
                {
                    //double phaseOffset = _phaseOffsetPitch;
                    for (int i = 0; i < mod_n; i++)
                    {
                        if (i > 1)//Not taking into account the first pitch and first yaw which should be 0
                        {
                            if (robot.modules[i].axis == "Pitch")
                                Dynamics.phaseDiff[i] = Dynamics.phaseDiff[i - 2] + _phaseOffsetPitch;
                            else
                                Dynamics.phaseDiff[i] = Dynamics.phaseDiff[i - 2] + _phaseOffsetYaw;
                        }
                    }
                }break;
            case "Forward":
                {
                    for (int i = 0; i < mod_n; i++)
                    {
                        if (i >= 1)//Not taking into account the first phase which should be 0
                        {
                                Dynamics.phaseDiff[i] = Dynamics.phaseDiff[i - 1] + _phaseOffsetPitch;
                            
                        }
                    }
                }break;

            case "Custom":
                {
                    for (int i = 0; i < mod_n; i++)
                    {
                        if (i > 1)//Not taking into account the first pitch and first yaw which should be 0
                        {
                            if (robot.modules[i].axis == "Pitch")
                                Dynamics.phaseDiff[i] = Dynamics.phaseDiff[i - 2] + _phaseOffsetPitch;
                            else
                                Dynamics.phaseDiff[i] = Dynamics.phaseDiff[i - 2] + _phaseOffsetYaw;
                        }
                    }
                }
                break;
        }

        Dynamics.currentAction = newAction;
    }
    public static void SetMovement(string command, double move_dir, double turn_dir)
    {
        action = command;
        if(move_dir != 0)
            move_direction = move_dir;
        if(turn_dir != 0)
            turn_direction = turn_dir;

        NewAction = true;

        //Debug.Log("Action: " + command);
    }
    public static void ChangeSpeed(double speed)
    {
        if (set_period - speed > 0 && set_period - speed < 20)
        {
            set_period -= speed;
            NewAction = true;
            //Debug.Log("Period: " + set_period);
        }
    }
    static double move_direction = 1;
    static double turn_direction = 1;
    static double set_period = 4.0f;
    static bool NewAction = false;
    public static bool Control(Simulation_Core.Robot robot, double t, double[] dyn_vars)
    {
        switch(action)
        {
            
            case "Forward":
                {
                    if (NewAction)
                        Initialize(action, robot, f_movementVars[0], f_movementVars[1], move_direction * f_movementVars[2], f_movementVars[3], f_movementVars[4], f_movementVars[5], f_movementVars[6]);
                    //Initialize(action, robot, (double)dyn_vars[0], 0, move_direction * (double)dyn_vars[2], 0, (double)dyn_vars[4], 0, 0);
                    Forward(robot, t);
                    return true;
                }
            case "Turn":
                {
                    if (NewAction)
                        Initialize(action, robot, t_movementVars[0], t_movementVars[1], move_direction * t_movementVars[2], t_movementVars[3], t_movementVars[4], t_movementVars[5], turn_direction * t_movementVars[6]);
                    //Initialize(action, robot, (double)dyn_vars[0], 0, move_direction * (double)dyn_vars[2], 0, (double)dyn_vars[4], 0, turn_direction * (double)dyn_vars[6]);

                    Turn(robot, t);
                    return true;
                }
            case "Idle": Idle();return true;

            case "Reset":Reset(robot, t);return true;

            default: return false;
        }
    }

    public static void Idle()
    {

    }
    public static void Reset(Simulation_Core.Robot robot, double t)
    {
        foreach (var mod in robot.modules)
        {
            mod.joint.Stabilize_Angle();
        }
    }

    public static void Forward(Simulation_Core.Robot robot, double t)//Standard forward movement
    {
        //Pitches:
        for (int i = 0; i<robot.modules.Count; i++)
        {
            if (robot.modules[i].axis == "Pitch")
            {

                angle = amplitudes[i] * (double)Math.Sin(2 * Math.PI * t / period[i] + phaseDiff[i]) + offset[i]; //Angle = amplitude + sin(2pi * t / period + phase diff) + offset
                robot.modules[i].joint.SetAngle(angle);
            }
            else
            {
                robot.modules[i].joint.SetAngle(0);
            }
        }
    }
    

    public static void Turn(Simulation_Core.Robot robot, double t)//Sideways rolling movement
    {

        for (int i = 0; i < robot.modules.Count; i++)
        {
            if (robot.modules[i].axis == "Pitch")
            {

                angle = amplitudes[i] * (double)Math.Sin(2 * Math.PI * t / period[i] + phaseDiff[i]) + offset[i]; //Angle = amplitude + sin(2pi * t / period + phase diff) + offset
                robot.modules[i].joint.SetAngle(angle);
            }
            //Reduntant, dont need the IFs
            if (robot.modules[i].axis == "Yaw")
            {

                angle = amplitudes[i] *(double) Math.Sin(2 * Math.PI * t / period[i] + phaseDiff[i]) + offset[i]; //Angle = amplitude + sin(2pi * t / period + phase diff) + offset
                robot.modules[i].joint.SetAngle(angle);
            }
        }
    }



}
