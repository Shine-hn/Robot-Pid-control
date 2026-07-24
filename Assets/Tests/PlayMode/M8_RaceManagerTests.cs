using System.Collections;
using NUnit.Framework;
using PIDReport.Control;
using PIDReport.Course;
using PIDReport.Race;
using PIDReport.Robot;
using UnityEngine;
using UnityEngine.TestTools;

namespace PIDReport.Tests
{
    public class M8_RaceManagerTests
    {
        private GameObject robot;
        private GameObject course;
        private GameObject extra;
        private GameObject extra2;

        [TearDown]
        public void TearDown()
        {
            if (robot != null) Object.Destroy(robot);
            if (course != null) Object.Destroy(course);
            if (extra != null) Object.Destroy(extra);
            if (extra2 != null) Object.Destroy(extra2);
        }

        private RaceManager BuildRobotWithRaceManager(Vector3 spawnPos, Quaternion spawnRot, bool useGravity = false)
        {
            robot = RobotFactory.CreateRobot(spawnPos, spawnRot);
            robot.GetComponent<RobotRig>().Body.useGravity = useGravity;
            return robot.AddComponent<RaceManager>();
        }

        [UnityTest]
        public IEnumerator DrivingIntoWall_InvalidatesWithWallReason()
        {
            course = CourseBuilder.BuildCourse();
            // Gravity + floor enabled here (unlike the other isolated tests): a real
            // wall collision needs the robot actually resting on the floor to respond
            // realistically. Without gravity, the collision impulse alone can spin the
            // robot up (no weight/floor contact damping it), tipping it over before the
            // wall-contact reason is even recorded.
            //
            // Spawn with room to reach a controlled speed before impact, and use a
            // moderate approach speed: with centerOfMass 0.5m above ~0.1m-high contact
            // points (per spec), even a legitimate off-axis collision impulse has a
            // large moment arm -- ramming the wall at full commanded speed while still
            // mid-acceleration-transient (as with a very close spawn) can genuinely tip
            // the robot, which is correct physical behavior but conflates two different
            // invalidation paths in what should be an isolated wall-contact test.
            var raceManager = BuildRobotWithRaceManager(new Vector3(0.50f, 0f, 1.50f), CourseBuilder.RobotSpawnRotation, useGravity: true);
            var drive = robot.AddComponent<DifferentialDriveController>();
            drive.SetWheelSpeeds(0.2f, 0.2f); // heads -X, straight toward WallWest

            bool invalidated = false;
            for (int i = 0; i < 150 && !invalidated; i++)
            {
                yield return new WaitForFixedUpdate();
                invalidated = raceManager.IsInvalidated;
            }

            Assert.IsTrue(invalidated, "Driving straight into a wall should invalidate the run.");
            StringAssert.Contains("wall", raceManager.InvalidationReason.ToLower());
        }

        [UnityTest]
        public IEnumerator SpawningOutsideCourseBounds_InvalidatesAsOffCourseBackstop()
        {
            var raceManager = BuildRobotWithRaceManager(new Vector3(-1f, 0f, 1.5f), Quaternion.identity);

            yield return new WaitForFixedUpdate();
            yield return new WaitForFixedUpdate();

            Assert.IsTrue(raceManager.IsInvalidated);
            StringAssert.Contains("outside", raceManager.InvalidationReason.ToLower());
        }

        [UnityTest]
        public IEnumerator RobotTippedOver_InvalidatesWithTipReason()
        {
            var raceManager = BuildRobotWithRaceManager(new Vector3(0f, 0.5f, 0f), Quaternion.identity);
            robot.GetComponent<RobotRig>().Body.useGravity = false;
            robot.GetComponent<RobotRig>().Body.AddTorque(Vector3.forward * 40f, ForceMode.Impulse);

            bool invalidated = false;
            for (int i = 0; i < 60 && !invalidated; i++)
            {
                yield return new WaitForFixedUpdate();
                invalidated = raceManager.IsInvalidated;
            }

            Assert.IsTrue(invalidated, "A large sideways torque should tip the robot over and invalidate the run.");
            StringAssert.Contains("tip", raceManager.InvalidationReason.ToLower());
        }

        [UnityTest]
        public IEnumerator StartLineTouch_StartsTheClock()
        {
            extra = BuildStandaloneLine("StartLine", new Vector3(0f, 0.6f, 0f), new Vector3(0.02f, 1.2f, 0.6f));
            var raceManager = BuildRobotWithRaceManager(new Vector3(0f, 0f, -0.5f), Quaternion.identity);
            var drive = robot.AddComponent<DifferentialDriveController>();
            drive.SetWheelSpeeds(0.3f, 0.3f); // heading +Z, straight into the line

            Assert.IsFalse(raceManager.RaceStarted, "Should not have started before touching the line.");

            bool started = false;
            for (int i = 0; i < 200 && !started; i++)
            {
                yield return new WaitForFixedUpdate();
                started = raceManager.RaceStarted;
            }

            Assert.IsTrue(started, "Touching the StartLine should start the clock.");
            Assert.Greater(raceManager.StartTime, 0f);
        }

        [UnityTest]
        public IEnumerator GoalLineFullCrossing_StopsTheClock()
        {
            // RaceManager only counts a finish once the race has actually started (matches
            // the brief: timing runs StartLine touch -> GoalLine full clearance, not just
            // "touched a GoalLine-tagged object in isolation") -- so this needs both lines
            // on the path, not just a GoalLine by itself.
            extra = BuildStandaloneLine("StartLine", new Vector3(0f, 0.6f, -0.2f), new Vector3(0.02f, 1.2f, 0.2f));
            extra2 = BuildStandaloneLine("GoalLine", new Vector3(0f, 0.6f, 0.4f), new Vector3(0.02f, 1.2f, 0.6f));
            var raceManager = BuildRobotWithRaceManager(new Vector3(0f, 0f, -0.5f), Quaternion.identity);
            var drive = robot.AddComponent<DifferentialDriveController>();
            drive.SetWheelSpeeds(0.3f, 0.3f); // heading +Z, drives straight through both lines

            bool finished = false;
            for (int i = 0; i < 450 && !finished; i++)
            {
                yield return new WaitForFixedUpdate();
                finished = raceManager.RaceFinished;
            }

            Assert.IsTrue(raceManager.RaceStarted, "Should have started via the StartLine on the way.");
            Assert.IsTrue(finished, "Fully crossing the GoalLine should stop the clock.");
            Assert.Greater(raceManager.FinishTime, raceManager.StartTime);
            Assert.Greater(raceManager.CourseTime, 0f);
        }

        private GameObject BuildStandaloneLine(string tag, Vector3 position, Vector3 size)
        {
            var go = new GameObject(tag);
            go.tag = tag;
            go.transform.position = position;
            var box = go.AddComponent<BoxCollider>();
            box.size = size;
            box.isTrigger = true;
            go.AddComponent<LineTrigger>();
            return go;
        }
    }
}
