using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using PIDReport.App;

namespace PIDReport.Editor
{
    // Creates the runnable scene asset and builds the standalone headful player for the
    // video deliverable (milestone M11). Both entry points are also usable headless via
    // -executeMethod so the whole pipeline runs from the CLI with no GUI interaction:
    //
    //   Unity.exe -batchmode -quit -projectPath <proj> \
    //       -executeMethod PIDReport.Editor.BuildScript.CreateMainScene
    //
    //   Unity.exe -batchmode -quit -projectPath <proj> \
    //       -executeMethod PIDReport.Editor.BuildScript.BuildStandalone -buildOutput <dir>
    public static class BuildScript
    {
        private const string ScenePath = "Assets/Scenes/Main.unity";

        // Idempotent: (re)creates Main.unity as a single GameObject carrying
        // SimulationRunner, which builds the entire course/robot/control stack at runtime.
        // Deliberately empty otherwise -- the camera and light are spawned by
        // SimulationRunner so the identical setup is guaranteed whether the scene is opened
        // manually or driven from a build.
        [MenuItem("Build/Create Main Scene")]
        public static void CreateMainScene()
        {
            Directory.CreateDirectory("Assets/Scenes");

            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var go = new GameObject("SimulationRunner");
            go.AddComponent<SimulationRunner>();

            bool saved = EditorSceneManager.SaveScene(scene, ScenePath);
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            if (!saved)
            {
                Debug.LogError("Failed to save scene at " + ScenePath);
                if (IsBatch()) EditorApplication.Exit(1);
                return;
            }

            Debug.Log("Created scene at " + ScenePath);
            if (IsBatch()) EditorApplication.Exit(0);
        }

        [MenuItem("Build/Build Standalone (Windows64)")]
        public static void BuildStandaloneMenu() => BuildStandalone();

        public static void BuildStandalone()
        {
            // Create the scene without exiting the editor (CreateMainScene would Exit(0) in
            // batch); inline the save here so the build can continue in the same invocation.
            Directory.CreateDirectory("Assets/Scenes");
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var runnerGo = new GameObject("SimulationRunner");
            runnerGo.AddComponent<SimulationRunner>();
            EditorSceneManager.SaveScene(scene, ScenePath);
            AssetDatabase.SaveAssets();

            // Every renderable object in this project is created at runtime via
            // GameObject.CreatePrimitive()/new Material(Shader.Find(...)), so NO asset in the
            // built scene references the Standard shader. Unity's build-time shader stripping
            // therefore drops it from the player, and every runtime-created material falls
            // back to the magenta "shader missing" error material. (This never shows in the
            // Editor, where all shaders are always loaded -- only in a standalone build.)
            // Forcing the shaders the runtime code depends on into Always-Included keeps them
            // in the player. This is why a from-code scene needs an explicit build step, not
            // just BuildPipeline.BuildPlayer on the scene.
            EnsureShaderIncluded("Standard");

            // Windowed, fixed 1280x720 so captured frames have a known, consistent size, and
            // keep simulating in the background so an unattended capture run never stalls.
            PlayerSettings.defaultScreenWidth = 1280;
            PlayerSettings.defaultScreenHeight = 720;
            PlayerSettings.fullScreenMode = FullScreenMode.Windowed;
            PlayerSettings.runInBackground = true;
            PlayerSettings.resizableWindow = false;

            string buildDir = GetArg("-buildOutput") ?? Path.GetFullPath("Build/Standalone");
            Directory.CreateDirectory(buildDir);
            string exePath = Path.Combine(buildDir, "PIDReportSim.exe");

            var options = new BuildPlayerOptions
            {
                scenes = new[] { ScenePath },
                locationPathName = exePath,
                target = BuildTarget.StandaloneWindows64,
                targetGroup = BuildTargetGroup.Standalone,
                options = BuildOptions.None
            };

            BuildReport report = BuildPipeline.BuildPlayer(options);
            BuildSummary summary = report.summary;

            Debug.Log("Build result=" + summary.result +
                      " outputPath=" + summary.outputPath +
                      " totalErrors=" + summary.totalErrors +
                      " totalSize=" + summary.totalSize);

            if (summary.result != BuildResult.Succeeded)
            {
                if (IsBatch()) EditorApplication.Exit(1);
                return;
            }

            if (IsBatch()) EditorApplication.Exit(0);
        }

        private static bool IsBatch() => Application.isBatchMode;

        // Adds a shader to Graphics Settings' Always-Included list so build-time stripping
        // can't drop it (idempotent). Needed for shaders referenced only from runtime code.
        // Unity 6 no longer exposes GraphicsSettings.alwaysIncludedShaders as a plain
        // property, so edit the m_AlwaysIncludedShaders array on the settings asset directly
        // via SerializedObject (stable across versions).
        private static void EnsureShaderIncluded(string shaderName)
        {
            var shader = Shader.Find(shaderName);
            if (shader == null)
            {
                Debug.LogWarning("EnsureShaderIncluded: shader not found: " + shaderName);
                return;
            }

            var settings = GraphicsSettings.GetGraphicsSettings();
            var so = new SerializedObject(settings);
            var arr = so.FindProperty("m_AlwaysIncludedShaders");

            for (int i = 0; i < arr.arraySize; i++)
            {
                if (arr.GetArrayElementAtIndex(i).objectReferenceValue == shader) return; // already present
            }

            int idx = arr.arraySize;
            arr.InsertArrayElementAtIndex(idx);
            arr.GetArrayElementAtIndex(idx).objectReferenceValue = shader;
            so.ApplyModifiedProperties();
            AssetDatabase.SaveAssets();
            Debug.Log("Added to Always-Included shaders: " + shaderName);
        }

        private static string GetArg(string name)
        {
            string[] args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
            {
                if (args[i] == name) return args[i + 1];
            }
            return null;
        }
    }
}
