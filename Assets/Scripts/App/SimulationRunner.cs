using System;
using System.Collections;
using System.Globalization;
using System.IO;
using UnityEngine;
using PIDReport.Robot;
using PIDReport.Course;
using PIDReport.Control;
using PIDReport.Race;
using PIDReport.Telemetry;
using PIDReport.Trajectory;

namespace PIDReport.App
{
    // Runtime entry point for a self-contained course run. Builds the exact same stack the
    // M10 regression test wires up (course, robot, camera-top kinematics, differential
    // drive, closed-loop trajectory tracking, race timing/invalidation, telemetry) -- but
    // as a playable, renderable scene rather than a headless test: it also spawns a camera
    // and light so the run is visible, draws an IMGUI HUD, and completes cleanly on its own
    // (writes the telemetry CSV, then stops).
    //
    // Because it drives itself start-to-finish with no user input and terminates on its
    // own, one scene serves three purposes:
    //   1. Press Play in the Editor to watch the run (manual visual check).
    //   2. A standalone headful build that auto-runs for video capture and calls
    //      Application.Quit() when done (milestone M11).
    //   3. A reproducible telemetry-CSV producer for the metrics deliverable.
    //
    // Command-line switches (standalone build; ignored harmlessly in the Editor):
    //   -capture              enable in-build PNG frame capture (see FrameCapture)
    //   -outdir <path>        directory for frames/ and telemetry.csv (default: persistentDataPath)
    //   -exitWhenDone <0|1>   override auto-quit (default 1 in a player, 0 in the Editor)
    public class SimulationRunner : MonoBehaviour
    {
        [Tooltip("Spawn a framing camera and directional light at startup (turn off if the scene already has its own).")]
        public bool BuildCameraAndLight = true;

        [Tooltip("Draw the IMGUI telemetry HUD overlay.")]
        public bool ShowHud = true;

        [Tooltip("Extra seconds to keep running after the race resolves, so the video has a short tail.")]
        public float FinishTailSeconds = 1.5f;

        [Tooltip("Hard wall-clock-independent safety margin added to the trajectory duration before force-stopping.")]
        public float TimeoutMarginSeconds = 5f;

        private GameObject course;
        private GameObject robot;
        private RaceManager raceManager;
        private TelemetryLogger telemetry;
        private TrajectoryTrackingController tracker;
        private CameraTopKinematics kinematics;
        private RobotTrajectory trajectory;
        private FrameCapture capture;

        private string outputDir;
        private bool captureEnabled;
        private bool exitWhenDone;
        private bool done;
        private string csvPath;
        private string summaryPath;

        void Start()
        {
            // Keep simulating even if the standalone window loses focus -- otherwise an
            // unattended capture run would stall the moment focus moves elsewhere.
            Application.runInBackground = true;

            ParseCommandLine();

            // Course must exist before the robot's components Awake: RaceManager finds the
            // Start/Goal line triggers by tag in its own Awake, and AddComponent runs Awake
            // synchronously. This ordering matches the M10 regression test exactly.
            course = CourseBuilder.BuildCourse();
            robot = RobotFactory.CreateRobot(CourseBuilder.RobotSpawnPosition, CourseBuilder.RobotSpawnRotation);

            kinematics = robot.AddComponent<CameraTopKinematics>();
            robot.AddComponent<DifferentialDriveController>();
            raceManager = robot.AddComponent<RaceManager>();
            telemetry = robot.AddComponent<TelemetryLogger>();
            tracker = robot.AddComponent<TrajectoryTrackingController>();

            trajectory = CourseTrajectoryPlanner.BuildCourseTrajectory();
            tracker.SetTrajectory(trajectory);

            StyleScene();
            if (BuildCameraAndLight) BuildViewpoint();

            if (captureEnabled)
            {
                capture = gameObject.AddComponent<FrameCapture>();
                capture.OutputDir = Path.Combine(outputDir, "frames");
                capture.Begin();
            }

            StartCoroutine(RunToCompletion());
        }

        private void ParseCommandLine()
        {
            outputDir = Application.persistentDataPath;
            // Default: quit automatically in a real player, but not when pressing Play in
            // the Editor (there, "quit" just ends play mode -- handled in Finish()).
            exitWhenDone = !Application.isEditor;

            string[] args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length; i++)
            {
                switch (args[i])
                {
                    case "-capture":
                        captureEnabled = true;
                        break;
                    case "-outdir":
                        if (i + 1 < args.Length) outputDir = args[++i];
                        break;
                    case "-exitWhenDone":
                        if (i + 1 < args.Length) exitWhenDone = args[++i] == "1";
                        break;
                }
            }

            Directory.CreateDirectory(outputDir);
            csvPath = Path.Combine(outputDir, "telemetry.csv");
            summaryPath = Path.Combine(outputDir, "run_summary.json");
        }

        // Values the telemetry CSV cannot express on its own. Most importantly 走破時間:
        // the CSV spans the WHOLE simulation (spawn runway + post-goal tail), so its last
        // timestamp is NOT the course time. Only RaceManager knows the StartLine-touch to
        // GoalLine-clearance interval, so it is published here for the metrics report
        // rather than being re-derived (incorrectly) downstream.
        [Serializable]
        private class RunSummary
        {
            public float courseTimeSeconds;
            public float startTimeSeconds;
            public float finishTimeSeconds;
            public float fixedTimestepSeconds;
            public float telemetryDurationSeconds;
            public bool raceStarted;
            public bool raceFinished;
            public bool invalidated;
            public string invalidationReason;
        }

        private IEnumerator RunToCompletion()
        {
            // Time budget is measured in sim time (fixedDeltaTime steps), independent of
            // wall clock and of Time.captureFramerate, so a slow capture run and a real-time
            // Editor run both terminate at the same simulated point.
            float maxSimTime = trajectory.TotalDuration + TimeoutMarginSeconds;
            float simTime = 0f;

            while (simTime < maxSimTime && !(raceManager.RaceFinished || raceManager.IsInvalidated))
            {
                yield return new WaitForFixedUpdate();
                simTime += Time.fixedDeltaTime;
            }

            // Short tail so the mp4 doesn't cut the instant the goal is crossed.
            float tail = FinishTailSeconds;
            while (tail > 0f)
            {
                yield return new WaitForFixedUpdate();
                tail -= Time.fixedDeltaTime;
            }

            Finish();
        }

        private void Finish()
        {
            if (done) return;
            done = true;

            if (capture != null) capture.End();

            telemetry.WriteCsv(csvPath);

            var summary = new RunSummary
            {
                courseTimeSeconds = raceManager.CourseTime,
                startTimeSeconds = raceManager.StartTime,
                finishTimeSeconds = raceManager.FinishTime,
                fixedTimestepSeconds = Time.fixedDeltaTime,
                telemetryDurationSeconds = telemetry.RowCount * Time.fixedDeltaTime,
                raceStarted = raceManager.RaceStarted,
                raceFinished = raceManager.RaceFinished,
                invalidated = raceManager.IsInvalidated,
                invalidationReason = raceManager.InvalidationReason ?? ""
            };
            File.WriteAllText(summaryPath, JsonUtility.ToJson(summary, true));

            Debug.Log("SimulationRunner finished: " +
                      "finished=" + raceManager.RaceFinished +
                      " invalidated=" + raceManager.IsInvalidated +
                      " reason=\"" + raceManager.InvalidationReason + "\"" +
                      " courseTime=" + raceManager.CourseTime.ToString("F3", CultureInfo.InvariantCulture) +
                      " maxCamTopAccel=" + telemetry.MaxCameraTopAcceleration.ToString("F4", CultureInfo.InvariantCulture) +
                      " frames=" + (capture != null ? capture.FrameCount : 0) +
                      " csv=" + csvPath);

            if (!exitWhenDone) return;

#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
#else
            Application.Quit();
#endif
        }

        // Elevated three-quarter view framing the whole ~2.4 x 3.0 m course from the south.
        private void BuildViewpoint()
        {
            var camGo = new GameObject("RunCamera");
            var cam = camGo.AddComponent<Camera>();
            cam.tag = "MainCamera";
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0.16f, 0.18f, 0.22f);
            cam.fieldOfView = 50f;
            Vector3 camPos = new Vector3(1.2f, 4.2f, -1.0f);
            Vector3 lookTarget = new Vector3(1.2f, 0.1f, 1.6f);
            camGo.transform.position = camPos;
            camGo.transform.rotation = Quaternion.LookRotation(lookTarget - camPos, Vector3.up);

            var lightGo = new GameObject("SunLight");
            var light = lightGo.AddComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = 1.1f;
            light.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
        }

        // Assigns distinct colored materials so the run is legible on video. Kept here in the
        // App layer (not in RobotFactory/CourseBuilder) so the tested build-geometry code
        // stays purely about physics/shape, with visual styling layered on top afterward.
        // All materials use the Standard shader, which the build step force-includes.
        private void StyleScene()
        {
            var floorMat = MakeMat(new Color(0.82f, 0.82f, 0.85f));
            var wallMat = MakeMat(new Color(0.30f, 0.42f, 0.62f));
            var blockMat = MakeMat(new Color(0.90f, 0.55f, 0.20f));
            var bodyMat = MakeMat(new Color(0.10f, 0.75f, 0.80f));
            var poleMat = MakeMat(new Color(0.25f, 0.25f, 0.28f));
            var headMat = MakeMat(new Color(0.95f, 0.85f, 0.15f));
            var wheelMat = MakeMat(new Color(0.10f, 0.10f, 0.12f));

            ColorizeChild(course.transform, "Floor", floorMat);
            ColorizeChild(course.transform, "WallSouth", wallMat);
            ColorizeChild(course.transform, "WallNorth", wallMat);
            ColorizeChild(course.transform, "WallWest", wallMat);
            ColorizeChild(course.transform, "WallEast", wallMat);
            ColorizeChild(course.transform, "LowerBlock", blockMat);
            ColorizeChild(course.transform, "UpperBlock", blockMat);

            ColorizeChild(robot.transform, "Body", bodyMat);
            ColorizeChild(robot.transform, "Pole", poleMat);
            ColorizeChild(robot.transform, "CameraHead", headMat);
            ColorizeChild(robot.transform, "WheelLeft", wheelMat);
            ColorizeChild(robot.transform, "WheelRight", wheelMat);
        }

        private static Material MakeMat(Color c)
        {
            return new Material(Shader.Find("Standard")) { color = c };
        }

        private static void ColorizeChild(Transform parent, string childName, Material mat)
        {
            var t = parent.Find(childName);
            if (t == null) return;
            var r = t.GetComponent<Renderer>();
            if (r != null) r.sharedMaterial = mat;
        }

        void OnGUI()
        {
            if (!ShowHud || raceManager == null) return;

            const int w = 340, pad = 10, line = 20;
            var boxStyle = GUI.skin.box;
            GUI.Box(new Rect(pad, pad, w, line * 9 + pad), GUIContent.none, boxStyle);

            var style = new GUIStyle(GUI.skin.label) { fontSize = 13, alignment = TextAnchor.MiddleLeft };
            style.normal.textColor = Color.white;

            int y = pad + 6;
            void Row(string s) { GUI.Label(new Rect(pad + 8, y, w - 16, line), s, style); y += line; }

            string state = raceManager.IsInvalidated ? "INVALIDATED"
                         : raceManager.RaceFinished ? "FINISHED"
                         : raceManager.RaceStarted ? "RUNNING" : "STAGING";

            float camAccel = kinematics != null ? kinematics.HorizontalAcceleration.magnitude : 0f;

            Row("State:        " + state);
            Row("Course time:  " + raceManager.CourseTime.ToString("F2") + " s");
            Row("Chassis speed:" + telemetry.MaxChassisSpeed.ToString("F2") + " m/s (max)");
            Row("Cam-top accel:" + camAccel.ToString("F2") + " m/s²  (cap 1.00)");
            Row("Cam-top max:  " + telemetry.MaxCameraTopAcceleration.ToString("F2") + " m/s²");
            Row("Long err:     " + tracker.LongitudinalError.ToString("F3") + " m");
            Row("Lat err:      " + tracker.LateralError.ToString("F3") + " m");
            Row("Head err:     " + (tracker.HeadingError * Mathf.Rad2Deg).ToString("F1") + " deg");
            if (raceManager.IsInvalidated)
                Row("Reason: " + raceManager.InvalidationReason);
        }
    }
}
