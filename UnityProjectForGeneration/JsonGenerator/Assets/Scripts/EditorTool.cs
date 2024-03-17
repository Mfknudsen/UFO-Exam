// ReSharper disable AccessToStaticMemberViaDerivedType
// ReSharper disable PossibleNullReferenceException

using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.AI;

public abstract class EditorTool
{
    [MenuItem("Tools/Run JSON Generator")]
    private static void JsonGenerator()
    {
        for (int i = 0; i < EditorSceneManager.loadedRootSceneCount; i++)
            EditorSceneManager.CloseScene(EditorSceneManager.GetSceneAt(i), false);

        string[] sceneNameLetter = { "S", "M", "L" }, folderName = { "Small", "Medium", "Large" };

        //Going from the Unity projects /Assets file path to /JsonFiles in the GitHub project for easier access for the C#, C++ and Python optimization scripts.
        string path = Application.dataPath;
        path = Directory.GetParent(path).FullName;
        path = Directory.GetParent(path).FullName;
        path = Directory.GetParent(path).FullName;
        path = $"{path}/JsonFiles/";
        Debug.Log(path);

        for (int sizeIndex = 0; sizeIndex < 3; sizeIndex++)
        for (int numberIndex = 1; numberIndex <= 5; numberIndex++)
        {
            string sceneName = $"{sceneNameLetter[sizeIndex]} {numberIndex}";
            EditorSceneManager.OpenScene($"Assets/Scenes/{folderName[sizeIndex]}/{sceneName}.unity");

            NavMeshTriangulation navMeshTriangulation = NavMesh.CalculateTriangulation();
            GameObject cleanPoint = GameObject.Find("Clean Point");

            NavMeshExport export = new NavMeshExport(cleanPoint.transform.position,
                navMeshTriangulation.vertices.ToList(), navMeshTriangulation.indices.ToList());

            Debug.Log(sceneName);
            Debug.Log($"Vertex count: {navMeshTriangulation.vertices.Length}");
            Debug.Log($"Index count:  {navMeshTriangulation.indices.Length}");

            string json = JsonUtility.ToJson(export);

            if (!Directory.Exists($"{path}{sceneName}.txt"))
                File.Create($"{path}{sceneName}.txt");

            File.WriteAllText($"{path}{sceneName}.txt", json);

            EditorSceneManager.CloseScene(EditorSceneManager.GetActiveScene(), false);
        }
    }

    private struct NavMeshExport
    {
        private Vector3 cleanPoint;
        private List<Vector3> vertices;
        private List<int> indices;

        public NavMeshExport(Vector3 cleanPoint, List<Vector3> vertices, List<int> indices)
        {
            this.cleanPoint = cleanPoint;
            this.vertices = vertices;
            this.indices = indices;
        }
    }
}