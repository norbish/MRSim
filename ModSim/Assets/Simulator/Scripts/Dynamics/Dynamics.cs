using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Dynamics
{
    public static float[] angles;
    public static float[] amplitudes;
    public static float[] period;
    public static float[] phaseDiff;
    public static float[] offset;

    static float angle; //rads
    public static void Forward(Simulation_Core.Robot robot, float t)//Standard forward movement
    {
        
        //Pitches:

        for (int i = 0; i<robot.modules.Count; i++)
        {
            if (robot.modules[i].Axis == "Pitch")
            {

                angle = amplitudes[i] + Mathf.Sin(2 * Mathf.PI * t / period[i] + phaseDiff[i]) + offset[i]; //Angle = amplitude + sin(2pi * t / period + phase diff) + offset
                robot.modules[i].joint.MOVE(angle);
            }
        }
    }
    

    public static void Turn(Simulation_Core.Robot robot, float t)//Sideways rolling movement
    {
        float angle; //rads
        //Pitches:

        for (int i = 0; i < robot.modules.Count; i++)
        {
            if (robot.modules[i].Axis == "Pitch")
            {

                angle = amplitudes[i] + Mathf.Sin(2 * Mathf.PI * t / period[i] + phaseDiff[i]) + offset[i]; //Angle = amplitude + sin(2pi * t / period + phase diff) + offset
                robot.modules[i].joint.MOVE(angle);
            }

            if (robot.modules[i].Axis == "Yaw")
            {

                angle = amplitudes[i] + Mathf.Sin(2 * Mathf.PI * t / period[i] + phaseDiff[i]) + offset[i]; //Angle = amplitude + sin(2pi * t / period + phase diff) + offset
                robot.modules[i].joint.MOVE(angle);
            }
        }
    }

    

}
