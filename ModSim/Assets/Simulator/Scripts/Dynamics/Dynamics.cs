using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Dynamics
{
    public static int mode = 1; //Must be changed if the script is customized. 
    public static string currentAction = "Idle";//check this string to see if have to initialize
    public static string action = "Idle";// = "Turn";
    public static float[] angles;
    public static float[] amplitudes;
    public static float[] period;
    public static float[] phaseDiff;
    public static float[] offset;

    static float angle; //rads

    public static void Initialize(string newAction, Simulation_Core.Robot robot, float _ampPitch,float _ampYaw, float _phaseOffsetPitch, float _phaseOffsetYaw, float _period, float _offsetPitch, float _offsetYaw)
    {
        Dynamics.action = newAction;
        int mod_n = robot.modules.Count;
        amplitudes = new float[mod_n];
        period = new float[mod_n];
        phaseDiff = new float[mod_n];
        offset = new float[mod_n];

        //can set "buttons" for each turning, rolling, etc.
        for (int i = 0; i < robot.modules.Count; i++)
        {
            Dynamics.period[i] = _period;

            if (robot.modules[i].Axis == "Pitch")
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
                    //float phaseOffset = _phaseOffsetPitch;
                    for (int i = 0; i < mod_n; i++)
                    {
                        if (i > 1)//Not taking into account the first pitch and first yaw which should be 0
                        {
                            if (robot.modules[i].Axis == "Pitch")
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
        }

        Dynamics.currentAction = newAction;
    }
    public static void SetMovement(string command, float move_dir, float turn_dir)
    {
        action = command;
        if(move_dir != 0)
            move_direction = move_dir;
        if(turn_dir != 0)
            turn_direction = turn_dir;

        NewAction = true;

        Debug.Log("Action: " + command);
    }
    public static void ChangeSpeed(float speed)
    {
        if (set_period - speed > 0 && set_period - speed < 20)
        {
            set_period -= speed;
            NewAction = true;
            Debug.Log("Period: " + set_period);
        }
    }
    static float move_direction = 1;
    static float turn_direction = 1;
    static float set_period = 4.0f;
    static bool NewAction = false;
    public static bool Control(Simulation_Core.Robot robot, float t)
    {
        /*if (Time.fixedTime >= 30)
            newAction = "Forward";
        else if (Time.fixedTime <= 25)
            newAction = "Turn";
        else
            newAction = "Reset";//Must have a smooth reset. */
        //newAction = "Forward";
        switch(action)
        {
            
            case "Forward":
                {
                    if (NewAction)
                        Initialize(action, robot, 2 * (Mathf.PI / 9.0f), 0, move_direction * 0.5f*Mathf.PI*2.0f/3.0f, 0, set_period, 0, 0);
                    Forward(robot, t);
                    return true;
                }
            case "Turn":
                {
                    if (NewAction)
                        Initialize(action, robot, 2 * (Mathf.PI / 9.0f), 0, move_direction * Mathf.PI * 2.0f / 3.0f, 0, set_period, 0, turn_direction * 20 * Mathf.PI / 180);


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
    public static void Reset(Simulation_Core.Robot robot, float t)
    {
        foreach (var mod in robot.modules)
        {
            mod.joint.Reset_Angle();
        }
    }

    public static void Forward(Simulation_Core.Robot robot, float t)//Standard forward movement
    {
        //Pitches:
        for (int i = 0; i<robot.modules.Count; i++)
        {
            if (robot.modules[i].Axis == "Pitch")
            {

                angle = amplitudes[i] * Mathf.Sin(2 * Mathf.PI * t / period[i] + phaseDiff[i]) + offset[i]; //Angle = amplitude + sin(2pi * t / period + phase diff) + offset
                robot.modules[i].joint.MOVE(angle);
            }
            else
            {
                robot.modules[i].joint.MOVE(0);
            }
        }
    }
    

    public static void Turn(Simulation_Core.Robot robot, float t)//Sideways rolling movement
    {

        for (int i = 0; i < robot.modules.Count; i++)
        {
            if (robot.modules[i].Axis == "Pitch")
            {

                angle = amplitudes[i] * Mathf.Sin(2 * Mathf.PI * t / period[i] + phaseDiff[i]) + offset[i]; //Angle = amplitude + sin(2pi * t / period + phase diff) + offset
                robot.modules[i].joint.MOVE(angle);
            }
            //Reduntant, dont need the IFs
            if (robot.modules[i].Axis == "Yaw")
            {

                angle = amplitudes[i] * Mathf.Sin(2 * Mathf.PI * t / period[i] + phaseDiff[i]) + offset[i]; //Angle = amplitude + sin(2pi * t / period + phase diff) + offset
                robot.modules[i].joint.MOVE(angle);
            }
        }
    }

}
