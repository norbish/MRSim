using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
public class Designer_Main : MonoBehaviour {

    public GameObject scrollArea;
    public GameObject Module_Alignment;
    RectTransform panel_rect;
    // Use this for initialization
    
	void Start ()
    {
        panel_rect = scrollArea.GetComponent<RectTransform>();

        int count = 0;
        Vector2 moduleBox_size = new Vector2(8.5f, 4);
        float height = 100;
        float offset = 0;

        List<Parameters> modules = new List<Parameters>();
        for(int i = 0; i<3; i++)
        {
            modules.Add(new Parameters()
            {
                name = "Module" + i,
                color = new Color(0, 0, i * 0.3f),//do gray/lightGray every 2nd module
                position = new Vector2(panel_rect.rect.center.x, panel_rect.rect.yMax - i * moduleBox_size.y * height),// panel_Content.GetComponent<RectTransform>().rect.center + new Vector2(0,235), //panel_Content.transform.position,//new Vector2(Module_Alignment.transform.position.x,0),
                size = moduleBox_size,
                parent = scrollArea,
            });
            
            
            modules[i].Create();
            offset += height * moduleBox_size.y;
            
        }
        //panel_rect.up = new Vector2(0,-offset);
        panel_rect.offsetMin = new Vector2(0,offset);
        //panel_rect.offsetMax = new Vector2(0, -offset);


        /*float width = mod1.panel.GetComponent<RectTransform>().rect.width;
        float ht = mod1.panel.GetComponent<RectTransform>().rect.height;*/

        count++;

        //panel_Content.GetComponent<RectTransform>().offsetMin = new Vector2(width, -ht * count);
        //panel_Content.GetComponent<RectTransform>().offsetMax = new Vector2 (width,ht*count);

        //panel.GetComponent<RectTransform>().transform.position = new Vector2(-600,0);

        //panelTransform.offsetMax = new Vector2(700,800);
        //panel.transform.SetParent(canvas.transform);
    }

    // Update is called once per frame
    void Update () {
		
	}
}
