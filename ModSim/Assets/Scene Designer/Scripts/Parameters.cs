using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Parameters {

    public GameObject panel;
    public string name;
    public Color color;
    public Vector2 position;
    public Vector2 size;
    public GameObject parent;
    private RectTransform rect;
    public Parameters()
    {

    }

    public void Create()
    {
        panel = new GameObject(name);
        rect = panel.AddComponent<RectTransform>();
        
        panel.transform.SetParent(parent.transform);
        panel.GetComponent<RectTransform>().transform.localScale = size;
        panel.transform.localPosition = position - new Vector2(0, 0);//(rect.rect.height /** size.y)/2*/));
        //panel.transform.position = position;// + new Vector2(135,-130);

        Image i = panel.AddComponent<Image>();
        i.color = color;
        //add textboxes, sliders, etc. 
    }

    public void AddVectorInput()
    {
        //add vector textboxes
    }
}
