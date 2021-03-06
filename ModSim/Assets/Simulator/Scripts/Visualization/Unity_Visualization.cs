﻿/* Unity Visulization of objects class (Unity_Visualization)
 * Torstein Sundnes Lenerand
 * NTNU Ålesund
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

namespace Unity_Visualization
{
    public static class Visualization//Class for checking if visualization is enabled
    {
        public static bool enabled = false;
    }

    public class SceneObject_Vis
    {
        public System.Guid guid;
        public GameObject gameobject;

        public SceneObject_Vis(System.Guid guid, Vector3 position, Vector3 scale, string shape)
        {
            this.guid = guid;
            switch(shape)
            {
                case "Box": gameobject = GameObject.CreatePrimitive(PrimitiveType.Cube);break;
                case "Sphere": gameobject = GameObject.CreatePrimitive(PrimitiveType.Sphere); break;
            }
            
            gameobject.name = "SceneObject";
            MeshRenderer renderer = gameobject.GetComponent<MeshRenderer>();

            if (guid == new System.Guid())
                renderer.material = new Material(Shader.Find("Transparent/Diffuse"));
            else
                renderer.material = new Material(Shader.Find("Diffuse"));

            renderer.material.color = Color.gray;

            gameobject.transform.localScale = scale * 2;
            gameobject.transform.position = position;
        }
        public void Update(Vector3 position, Vector3 rotation)//designer
        {
            gameobject.transform.position = position;
            gameobject.transform.eulerAngles = rotation;
            //gameobject.transform.localScale = size * 2;//*2 for scaling up to AgX size
        }
        public void Update(Vector3 position, Quaternion rotation)
        {
            gameobject.transform.position = position;
            gameobject.transform.rotation = rotation;
        }
        public void Remove()
        {
            GameObject.Destroy(gameobject.gameObject);
            //this.Dispose();
        }
    }

    public class ForceSensor_Vis
    {
        public System.Guid guid;
        public GameObject gameobject;

        public ForceSensor_Vis(System.Guid guid, Vector3 position, Vector3 scale)
        {
            this.guid = guid;
            gameobject = GameObject.CreatePrimitive(PrimitiveType.Cube);
            gameobject.name = "ForceSensor";
            MeshRenderer renderer = gameobject.GetComponent<MeshRenderer>();

            renderer.material = new Material(Shader.Find("Diffuse"))
            {
                color = Color.red
            };

            gameobject.transform.localScale = scale * 2;
            gameobject.transform.position = position;
        }
        public void Update(Vector3 position, Quaternion rotation)
        {
            gameobject.transform.position = position;
            gameobject.transform.rotation = rotation;
        }
        public void Remove()
        {
            GameObject.Destroy(gameobject.gameObject);
            //this.Dispose();
        }
    }

    public class SensorModule_Vis
    {
        public System.Guid guid;
        public GameObject gameobject;

        public SensorModule_Vis(System.Guid guid, Vector3 position, Vector3 scale)
        {
            this.guid = guid;
            gameobject = GameObject.CreatePrimitive(PrimitiveType.Cube);
            gameobject.name = "SensorModule";
            MeshRenderer renderer = gameobject.GetComponent<MeshRenderer>();

            if (guid == new System.Guid())
                renderer.material = new Material(Shader.Find("Transparent/Diffuse"));
            else
                renderer.material = new Material(Shader.Find("Diffuse"));

            renderer.material.color = Color.white;

            gameobject.transform.localScale = scale*2;
            gameobject.transform.position = position;
        }
        public void Update(Vector3 position, Quaternion rotation)
        {
            gameobject.transform.position = position;
            gameobject.transform.rotation = rotation;
        }
        public void Update(Vector3 position, Vector3 rotation, Vector3 size)//designer
        {
            gameobject.transform.position = position;
            gameobject.transform.eulerAngles = rotation;
            gameobject.transform.localScale = size*2;//*2 for scaling up to AgX size
        }
        public void Remove()
        {
            GameObject.Destroy(gameobject.gameObject);
            //this.Dispose();
        }
    }

    public class Frame_Vis
    {
        public System.Guid guid;
        //public Mesh mesh;//mesh of the frame
        public GameObject gameobject;//Have to store the mesh in some way

        public Frame_Vis(System.Guid guid, Mesh meshFilter, Vector3 initialpos, double scale)
        {
            this.guid = guid;

            gameobject = new GameObject("Frame");

            MeshRenderer renderer = gameobject.AddComponent<MeshRenderer>();
            MeshFilter filter = gameobject.AddComponent<MeshFilter>();

            Mesh tmpMesh = new Mesh();
            tmpMesh = meshFilter;

            Vector3[] tmp_Vertices = tmpMesh.vertices;
            for (int i = 0; i < tmp_Vertices.Length; i++)
            {
                tmp_Vertices[i].x *= (float)scale;
                tmp_Vertices[i].y *= (float)scale;
                tmp_Vertices[i].z *= (float)scale;
            }
            tmpMesh.vertices = tmp_Vertices;

            tmpMesh.RecalculateBounds();

            filter.mesh = tmpMesh;
            if(guid == new System.Guid())
                renderer.material = new Material(Shader.Find("Transparent/Diffuse"));
            else
                renderer.material = new Material(Shader.Find("Diffuse"));

            renderer.material.color =  Color.blue;

            gameobject.transform.position = initialpos;//gameobject.AddComponent<Renderer>();

        }

        public void Update(Vector3 position/*, Vector3 scale,*/ ,Quaternion rotation, string axis)
        {
            gameobject.transform.position = position;
            //gameobject.transform.localScale = scale;
            gameobject.transform.rotation = rotation;
            gameobject.GetComponent<MeshRenderer>().material.color = axis == "Pitch" ? Color.gray : Color.blue;
        }
        public void Update(Vector3 position, Vector3 rotation, string axis)//designer
        {
            gameobject.transform.position = position;
            //gameobject.transform.localScale = size;
            gameobject.transform.eulerAngles = rotation;
            Color color =  axis == "Pitch" ? Color.gray : Color.blue;
            color.a = 0.3f;
            gameobject.GetComponent<MeshRenderer>().material.color = color;
        }

        public void Remove()
        {
            GameObject.Destroy(gameobject.gameObject);
            //this.Dispose();
        }
    }

    public class Joint_Vis
    {
        public System.Guid guid;
        public string jointType;
        public GameObject left_p;
        public GameObject mid_p;
        public GameObject right_p;
        
        public Joint_Vis(System.Guid guid)
        {
            this.guid = guid;
            //left_p = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            mid_p = GameObject.CreatePrimitive(PrimitiveType.Sphere);
           // right_p = GameObject.CreatePrimitive(PrimitiveType.Sphere);

        }
        public void Update(Vector3[] contactPoints)
        {
           // left_p.transform.position = contactPoints[0];
            mid_p.transform.position = contactPoints[1];
           // right_p.transform.position = contactPoints[2];
        }
    }

    public class Scene_Vis
    {
        
        public System.Guid guid;
        public Mesh mesh;
        public GameObject terrain;
        public Scene_Vis(System.Guid guid, List<Vector3> vertices, List<int> triangles, Vector2[] uvs, Vector3 position, Texture texture)//REMOVE
        {
            //Assign to the mesh, Unity:
            UnityEngine.Mesh mesh = new Mesh
            {
                vertices = vertices.ToArray(),
                uv = uvs,
                triangles = triangles.ToArray()
            };
            mesh.RecalculateBounds();
            mesh.RecalculateNormals();
            terrain = GameObject.CreatePrimitive(PrimitiveType.Cube);//Create the primitive plane 
            terrain.name = "Terrain";
            MeshRenderer renderer = terrain.GetComponent<MeshRenderer>();

            renderer.material = new Material(Shader.Find("Diffuse"));
            renderer.material.SetTexture("_MainTex", texture);
            renderer.material.mainTextureScale = new Vector2(0.1f,0.1f);

            terrain.GetComponent<MeshFilter>().sharedMesh = mesh;
            //mesh = terrain.GetComponent<MeshFilter>().mesh;

            terrain.transform.position = position;
            //heightmapCube.transform.rotation = );// new Vector3(0, 0, 90);//rotate to match AgX
        }

        public void Remove()
        {
            GameObject.Destroy(terrain.gameObject);
        }
    }
}
