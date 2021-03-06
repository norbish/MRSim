﻿/* Analytics, saving robot variables class (Analytics_Visualization)
 * Torstein Sundnes Lenerand
 * NTNU Ålesund
 */

using System.Collections;
using System.Collections.Generic;
using System.IO;
using Simulation_Core;

public static class Analytics_Visualization {

    public static double Interval = 1;
    public static bool Enabled = true;
	// Use this for initialization
	static void Start () {
		
	}

    // Update is called once per frame

    static Dictionary<string, bool> checklist = new Dictionary<string, bool>()
    {
        {"position",true},
        {"angle",true},
        {"distance",true },
        {"force",true },
    };
    static double updatedTime = 0;

    /*-------------------------------------Initial setup---------------------*/
    public static int SaveData(Robot robot,double sim_Time/*Seconds*/)
    {
        //check which checkboxes in the analytics popup window are selected, and for(i=0-20?) save the data needed: UpdateCheckboxesChecked()
        if (sim_Time - updatedTime < Interval)//if interval not reached:
            return 0;//abort

        string data = sim_Time.ToString();

        //Modules:
        foreach (Module mod in robot.modules)//one module at a time.
            foreach (KeyValuePair<string, bool> field in checklist)//each vis item checked.
            {
                if (field.Value == true)//if checked
                {
                    switch (field.Key)
                    {
                        case "position": data += ";" + mod.position.x + ";" + mod.position.y + ";" + mod.position.z; break;//position value + break: ";"
                        case "angle": data += ";" + mod.joint.GetAngle(); break;//joint angle + break: ";"
                    }
                }
            }
        //Sensory modules:
        foreach(SensorModule mod in robot.sensorModules)
        {
            foreach (KeyValuePair<string, bool> field in checklist)//each vis item checked.
            {
                if (field.Value == true)//if checked
                {
                    switch (field.Key)
                    {
                        case "distance":
                            {
                                foreach(DistanceSensor ds in mod.distanceSensors)
                                    data += ";" + ds.GetSensorDistance();

                                break;
                            }
                        case "force": if (mod.forceSensor != null) { data += ";" + mod.forceSensor.forceValue; } break;//joint angle + break: ";"
                    }
                }
            }
        }
        //for each checkarray item:
        //if item.checked
        //datastring += datavalue[i]+";"


        int result = SaveToFile(data,robot);
        if (result == 1)
        {
            updatedTime = sim_Time;//Last update time
            return 1;//(might wanna do this only once a min or something).
        }
        else return result;
    }

    static void UpdateCheckboxesChecked()
    {
        //update the array containing which data to be updated

    }
    public static string Input_Filename = @"";
    public static int SaveToFile(string data,Robot robot)
    {
        if (Input_Filename != "")
        {
            //Application.streamingAssetsPath + "/XML/Scenario.xml";// Add location to project (system... function) and + streamingasset path. 
            string path = Input_Filename;
            //StreamWriter writer = new StreamWriter(path);
            try
            {
                if (!File.Exists(path))//FIRST TIME
                {
                    File.Create(path).Dispose();
                    var tw = new StreamWriter(path);

                    string headers = "Time";
                    //Module headers
                    for (int i = 0; i < robot.modules.Count; i++)
                        foreach (KeyValuePair<string, bool> field in checklist)//each check item
                        {
                            if (field.Value == true)//if checked
                            {
                                switch (field.Key)
                                {
                                    case "position": headers += ";mod" + i + " Pos.x"+ ";mod" + i + " Pos.y"+ ";mod" + i + " Pos.z"; break;//position value + break: ";"
                                    case "angle": headers += ";mod" + i + " Angle"; break;//joint angle + break: ";"
                                }
                            }
                        }
                    //sensor module headers
                    for (int i = 0; i < robot.sensorModules.Count; i++)
                    {
                        foreach (KeyValuePair<string, bool> field in checklist)//each check item
                        {
                            if (field.Value == true)//if checked
                            {
                                switch (field.Key)
                                {
                                    case "distance":
                                        {
                                            foreach (DistanceSensor ds in robot.sensorModules[i].distanceSensors)
                                                headers += ";" + "sMod"+i + " distanceSensor" + ds.sensorPosition;

                                            break;
                                        }
                                    case "force": if (robot.sensorModules[i].forceSensor != null) { headers += ";" + "sMod" + i + " forceSensor"; } break;//joint angle + break: ";"
                                }
                            }
                        }
                    }
                   

                    tw.WriteLine(headers);
                    tw.WriteLine(data);
                    tw.Close();
                }
                else if (File.Exists(path))//nTH TIME
                {
                    using (var tw = new StreamWriter(path, true))
                    {
                        tw.WriteLine(data);
                        tw.Close();
                    }
                }
            }catch(IOException)//Folder not created?
            {
                return 6;
            }
            return 1;

            // write a line of text to the file
            //writer.Write(data);//or writeline
            //writer.Close();  //or tw.Flush();
        }return 7;
    }
}
