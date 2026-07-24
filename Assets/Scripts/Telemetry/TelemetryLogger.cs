using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;
using PIDReport.Robot;
using PIDReport.Trajectory;

namespace PIDReport.Telemetry
{
    // Per-FixedUpdate telemetry capture: position, heading, chassis and camera-top
    // speed/acceleration/jerk, angular velocity. Buffered in memory and written to CSV
    // on demand (WriteCsv), so this is usable both as the deliverable-#2 data source and
    // as a reusable logger for any run, not a one-off script tied to a specific report.
    [RequireComponent(typeof(RobotRig))]
    public class TelemetryLogger : MonoBehaviour
    {
        private struct Row
        {
            public float Time;
            public float PosX, PosZ;
            public float HeadingDeg;
            public float ChassisSpeed;
            public float ChassisAccel;
            public float CameraTopSpeed;
            public float CameraTopAccel;
            public float CameraTopJerk;
            public float AngularVelocityDeg;
        }

        private RobotRig rig;
        private CameraTopKinematics kinematics;

        private readonly List<Row> rows = new List<Row>();
        private Vector3 previousChassisVelocity;
        private bool hasPreviousChassisVelocity;
        private float elapsed;

        public int RowCount => rows.Count;
        public float MaxChassisSpeed { get; private set; }
        public float MaxCameraTopSpeed { get; private set; }
        public float MaxCameraTopAcceleration { get; private set; }
        public float AverageCameraTopAcceleration { get; private set; }
        public float MaxCameraTopJerk { get; private set; }
        public float MaxAngularSpeedDeg { get; private set; }

        void Awake()
        {
            rig = GetComponent<RobotRig>();
            kinematics = GetComponent<CameraTopKinematics>();
        }

        void FixedUpdate()
        {
            Vector3 chassisVelocity = rig.Body.linearVelocity;
            chassisVelocity.y = 0f;

            float chassisAccel = 0f;
            if (hasPreviousChassisVelocity)
            {
                chassisAccel = (chassisVelocity - previousChassisVelocity).magnitude / Time.fixedDeltaTime;
            }
            previousChassisVelocity = chassisVelocity;
            hasPreviousChassisVelocity = true;

            float cameraTopSpeed = kinematics != null ? kinematics.HorizontalVelocity.magnitude : 0f;
            float cameraTopAccel = kinematics != null ? kinematics.HorizontalAcceleration.magnitude : 0f;
            float cameraTopJerk = kinematics != null ? kinematics.HorizontalJerk.magnitude : 0f;
            float angularSpeedDeg = Mathf.Abs(rig.Body.angularVelocity.y) * Mathf.Rad2Deg;

            rows.Add(new Row
            {
                Time = elapsed,
                PosX = rig.Body.transform.position.x,
                PosZ = rig.Body.transform.position.z,
                HeadingDeg = HeadingUtil.FromForward(rig.Body.transform.forward) * Mathf.Rad2Deg,
                ChassisSpeed = chassisVelocity.magnitude,
                ChassisAccel = chassisAccel,
                CameraTopSpeed = cameraTopSpeed,
                CameraTopAccel = cameraTopAccel,
                CameraTopJerk = cameraTopJerk,
                AngularVelocityDeg = angularSpeedDeg
            });

            MaxChassisSpeed = Mathf.Max(MaxChassisSpeed, chassisVelocity.magnitude);
            MaxCameraTopSpeed = Mathf.Max(MaxCameraTopSpeed, cameraTopSpeed);
            MaxCameraTopAcceleration = Mathf.Max(MaxCameraTopAcceleration, cameraTopAccel);
            MaxCameraTopJerk = Mathf.Max(MaxCameraTopJerk, cameraTopJerk);
            MaxAngularSpeedDeg = Mathf.Max(MaxAngularSpeedDeg, angularSpeedDeg);

            float total = 0f;
            foreach (var r in rows) total += r.CameraTopAccel;
            AverageCameraTopAcceleration = rows.Count > 0 ? total / rows.Count : 0f;

            elapsed += Time.fixedDeltaTime;
        }

        public void WriteCsv(string filePath)
        {
            string dir = Path.GetDirectoryName(filePath);
            if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);

            using (var writer = new StreamWriter(filePath, false))
            {
                writer.WriteLine("time_s,pos_x_m,pos_z_m,heading_deg,chassis_speed_mps,chassis_accel_mps2," +
                                  "camera_top_speed_mps,camera_top_accel_mps2,camera_top_jerk_mps3,angular_speed_degps");
                foreach (var r in rows)
                {
                    writer.WriteLine(string.Join(",",
                        F(r.Time), F(r.PosX), F(r.PosZ), F(r.HeadingDeg),
                        F(r.ChassisSpeed), F(r.ChassisAccel),
                        F(r.CameraTopSpeed), F(r.CameraTopAccel), F(r.CameraTopJerk),
                        F(r.AngularVelocityDeg)));
                }
            }
        }

        private static string F(float value) => value.ToString("G6", CultureInfo.InvariantCulture);
    }
}
