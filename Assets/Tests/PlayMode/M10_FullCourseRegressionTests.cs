using System.Collections;
using System.IO;
using NUnit.Framework;
using PIDReport.Control;
using PIDReport.Course;
using PIDReport.Race;
using PIDReport.Robot;
using PIDReport.Telemetry;
using PIDReport.Trajectory;
using UnityEngine;
using UnityEngine.TestTools;

namespace PIDReport.Tests
{
    // The master gate: drives the entire confirmed course with the real stack (course
    // geometry, robot, camera-top kinematics, differential drive, closed-loop trajectory
    // tracking, race timing/invalidation, telemetry) wired together exactly as the real
    // race would, and checks every hard requirement from the brief at once.
    public class M10_FullCourseRegressionTests
    {
        private GameObject course;
        private GameObject robot;
        private string tempCsvPath;

        [TearDown]
        public void TearDown()
        {
            if (robot != null) Object.Destroy(robot);
            if (course != null) Object.Destroy(course);
            if (tempCsvPath != null && File.Exists(tempCsvPath)) File.Delete(tempCsvPath);
        }

        [UnityTest]
        public IEnumerator FullCourse_CompletesWithoutInvalidation_AndStaysUnderAccelerationCap()
        {
            course = CourseBuilder.BuildCourse();
            robot = RobotFactory.CreateRobot(CourseBuilder.RobotSpawnPosition, CourseBuilder.RobotSpawnRotation);

            robot.AddComponent<CameraTopKinematics>();
            robot.AddComponent<DifferentialDriveController>();
            var raceManager = robot.AddComponent<RaceManager>();
            var telemetry = robot.AddComponent<TelemetryLogger>();
            var tracker = robot.AddComponent<TrajectoryTrackingController>();

            var trajectory = CourseTrajectoryPlanner.BuildCourseTrajectory();
            tracker.SetTrajectory(trajectory);

            int maxSteps = Mathf.CeilToInt((trajectory.TotalDuration + 5f) / Time.fixedDeltaTime);
            int stepsRun = 0;
            for (int i = 0; i < maxSteps; i++)
            {
                yield return new WaitForFixedUpdate();
                stepsRun++;
                if (raceManager.IsInvalidated || raceManager.RaceFinished) break;
            }

            Assert.IsFalse(raceManager.IsInvalidated,
                "Run was invalidated: " + raceManager.InvalidationReason + " (after " + stepsRun + " steps)");
            Assert.IsTrue(raceManager.RaceFinished,
                "Robot did not reach and fully clear the GoalLine within the expected time budget (" + stepsRun + " steps run).");
            Assert.Greater(raceManager.CourseTime, 0f);

            Assert.LessOrEqual(telemetry.MaxCameraTopAcceleration, 1.0f,
                "Camera-top horizontal resultant acceleration exceeded the 1.00 m/s^2 hard cap: " +
                telemetry.MaxCameraTopAcceleration);

            Assert.Greater(telemetry.RowCount, 0);

            tempCsvPath = Path.Combine(Application.temporaryCachePath, "m10_full_course_" + System.Guid.NewGuid().ToString("N") + ".csv");
            telemetry.WriteCsv(tempCsvPath);
            Assert.IsTrue(File.Exists(tempCsvPath));
            Assert.Greater(new FileInfo(tempCsvPath).Length, 0);
        }
    }
}
