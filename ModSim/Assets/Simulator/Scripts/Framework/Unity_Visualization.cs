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

    public class Sensor_Vis
    {
        public System.Guid guid;
        public GameObject gameobject;

        public Sensor_Vis(System.Guid guid, Vector3 position, Vector3 scale)
        {
            this.guid = guid;
            gameobject = GameObject.CreatePrimitive(PrimitiveType.Cube);
            MeshRenderer renderer = gameobject.GetComponent<MeshRenderer>();


            renderer.material = new Material(Shader.Find("Transparent/Diffuse"));
            renderer.material.color = Color.white;

            gameobject.transform.localScale = scale*2;
            gameobject.transform.position = position;
        }
        public void Update(Vector3 position, Vector3 rotation)
        {
            gameobject.transform.position = position;
            gameobject.transform.eulerAngles = rotation;
        }
    }

    public class Frame_Vis
    {
        public System.Guid guid;
        //public Mesh mesh;//mesh of the frame
        public GameObject gameobject;//Have to store the mesh in some way

        public Frame_Vis(System.Guid guid, Mesh meshFilter, Vector3 initialpos, float scale)
        {
            this.guid = guid;

            gameobject = new GameObject("Frame");

            MeshRenderer renderer = gameobject.AddComponent<MeshRenderer>();
            MeshFilter filter = gameobject.AddComponent<MeshFilter>();

            Vector3[] tmp_Vertices = meshFilter.vertices;
            for (int i = 0; i < tmp_Vertices.Length; i++)
            {
                tmp_Vertices[i].x *= scale;
                tmp_Vertices[i].y *= scale;
                tmp_Vertices[i].z *= scale;
            }
            meshFilter.vertices = tmp_Vertices;

            filter.mesh = meshFilter;
            renderer.material = new Material(Shader.Find("Diffuse"));
            renderer.material.color =  Color.blue;

            gameobject.transform.position = initialpos;//gameobject.AddComponent<Renderer>();

        }

        public void Update(Vector3 position/*, Vector3 scale,*/ ,Vector3 rotation, string axis)
        {
            gameobject.transform.position = position;
            //gameobject.transform.localScale = scale;
            gameobject.transform.eulerAngles = rotation;
            gameobject.GetComponent<MeshRenderer>().material.color = axis == "Pitch" ? Color.gray : Color.blue;
        }
    }

    public class Joint_Vis
    {
        public System.Guid guid;
        public string jointType;
        public GameObject left_p;
        public GameObject mid_p;
        public GameObject right_p;
        Vector3[] contactPoints = new Vector3[3];//left attachment point, middle of joint, and right attachment point. 

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

            terrain = GameObject.CreatePrimitive(PrimitiveType.Cube);//Create the primitive plane 
            terrain.name = "Terrain";
            MeshRenderer renderer = terrain.GetComponent<MeshRenderer>();

            renderer.material = new Material(Shader.Find("Transparent/Diffuse"));
            renderer.material.SetTexture("_MainTex", texture);

            terrain.GetComponent<MeshFilter>().sharedMesh = mesh;
            mesh = terrain.GetComponent<MeshFilter>().mesh;

            terrain.transform.position = position;
            //heightmapCube.transform.rotation = );// new Vector3(0, 0, 90);//rotate to match AgX
        }
    }
}
