using System.Collections;
using System.IO;
using NUnit.Framework;
using PIDReport.Control;
using PIDReport.Robot;
using PIDReport.Telemetry;
using UnityEngine;
using UnityEngine.TestTools;

namespace PIDReport.Tests
{
    public class M9_TelemetryLoggerTests
    {
        private GameObject robot;
        private string tempCsvPath;

        [TearDown]
        public void TearDown()
        {
            if (robot != null) Object.Destroy(robot);
            if (tempCsvPath != null && File.Exists(tempCsvPath)) File.Delete(tempCsvPath);
        }

        [UnityTest]
        public IEnumerator TelemetryLogger_RecordsOneRowPerFixedUpdate()
        {
            robot = RobotFactory.CreateRobot(Vector3.zero, Quaternion.identity);
            robot.GetComponent<RobotRig>().Body.useGravity = false;
            robot.AddComponent<CameraTopKinematics>();
            robot.AddComponent<DifferentialDriveController>();
            var telemetry = robot.AddComponent<TelemetryLogger>();

            for (int i = 0; i < 40; i++) yield return new WaitForFixedUpdate();

            Assert.AreEqual(40, telemetry.RowCount);
        }

        [UnityTest]
        public IEnumerator TelemetryLogger_SummaryStatsReflectForwardDrive()
        {
            robot = RobotFactory.CreateRobot(Vector3.zero, Quaternion.identity);
            robot.GetComponent<RobotRig>().Body.useGravity = false;
            robot.AddComponent<CameraTopKinematics>();
            var drive = robot.AddComponent<DifferentialDriveController>();
            var telemetry = robot.AddComponent<TelemetryLogger>();
            drive.SetWheelSpeeds(0.3f, 0.3f);

            for (int i = 0; i < 100; i++) yield return new WaitForFixedUpdate();

            Assert.Greater(telemetry.MaxChassisSpeed, 0.1f, "Should have recorded meaningful chassis speed while driving forward.");
            Assert.Greater(telemetry.MaxCameraTopSpeed, 0.1f);
            Assert.GreaterOrEqual(telemetry.MaxCameraTopAcceleration, telemetry.AverageCameraTopAcceleration,
                "Max should never be less than the average.");
        }

        [UnityTest]
        public IEnumerator WriteCsv_ProducesParseableFileWithCorrectRowCount()
        {
            robot = RobotFactory.CreateRobot(Vector3.zero, Quaternion.identity);
            robot.GetComponent<RobotRig>().Body.useGravity = false;
            robot.AddComponent<CameraTopKinematics>();
            robot.AddComponent<DifferentialDriveController>();
            var telemetry = robot.AddComponent<TelemetryLogger>();

            for (int i = 0; i < 25; i++) yield return new WaitForFixedUpdate();

            tempCsvPath = Path.Combine(Application.temporaryCachePath, "telemetry_test_" + System.Guid.NewGuid().ToString("N") + ".csv");
            telemetry.WriteCsv(tempCsvPath);

            Assert.IsTrue(File.Exists(tempCsvPath));
            string[] lines = File.ReadAllLines(tempCsvPath);

            Assert.AreEqual(26, lines.Length, "Expected 1 header line + 25 data rows.");
            StringAssert.StartsWith("time_s,", lines[0]);

            string[] firstDataRow = lines[1].Split(',');
            Assert.AreEqual(10, firstDataRow.Length, "Expected 10 columns per row.");
            Assert.DoesNotThrow(() => float.Parse(firstDataRow[0], System.Globalization.CultureInfo.InvariantCulture));
        }
    }
}
