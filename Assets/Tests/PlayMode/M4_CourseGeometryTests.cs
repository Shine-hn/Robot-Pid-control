using System.Collections;
using NUnit.Framework;
using PIDReport.Course;
using PIDReport.Robot;
using UnityEngine;
using UnityEngine.TestTools;

namespace PIDReport.Tests
{
    public class M4_CourseGeometryTests
    {
        private GameObject course;
        private GameObject robot;

        [TearDown]
        public void TearDown()
        {
            if (course != null) Object.Destroy(course);
            if (robot != null) Object.Destroy(robot);
        }

        [Test]
        public void BuildCourse_AllTagsResolveToExpectedObjectCounts()
        {
            course = CourseBuilder.BuildCourse();

            // This indirectly re-verifies the tags registered in TagManager.asset are both
            // present AND actually applied to the right objects -- a tag that "looked"
            // assigned in the Editor but wasn't would show up here as a wrong/zero count,
            // same failure mode as a silently-mistyped tag string.
            var walls = GameObject.FindGameObjectsWithTag("Wall");
            Assert.AreEqual(6, walls.Length, "Expected all 6 walls/blocks to carry the Wall tag.");

            var startLine = GameObject.FindGameObjectWithTag("StartLine");
            Assert.IsNotNull(startLine, "StartLine object with the StartLine tag was not found.");

            var goalLine = GameObject.FindGameObjectWithTag("GoalLine");
            Assert.IsNotNull(goalLine, "GoalLine object with the GoalLine tag was not found.");
        }

        [Test]
        public void BuildCourse_StartAndGoalLineTriggersAreAtSpecPositions()
        {
            course = CourseBuilder.BuildCourse();
            Physics.SyncTransforms(); // bounds read below is same-frame as the transforms above

            var startLine = GameObject.FindGameObjectWithTag("StartLine");
            var startBounds = startLine.GetComponent<Collider>().bounds;
            Assert.AreEqual(0.60f, startBounds.center.x, 0.001f);
            Assert.AreEqual(0f, startBounds.min.z, 0.01f);
            Assert.AreEqual(0.6f, startBounds.max.z, 0.01f);

            var goalLine = GameObject.FindGameObjectWithTag("GoalLine");
            var goalBounds = goalLine.GetComponent<Collider>().bounds;
            Assert.AreEqual(1.80f, goalBounds.center.x, 0.001f);
            Assert.AreEqual(2.4f, goalBounds.min.z, 0.01f);
            Assert.AreEqual(3.0f, goalBounds.max.z, 0.01f);
        }

        [Test]
        public void RobotSpawn_HasClearanceFromLowerBlock()
        {
            course = CourseBuilder.BuildCourse();
            robot = RobotFactory.CreateRobot(CourseBuilder.RobotSpawnPosition, CourseBuilder.RobotSpawnRotation);

            // Same-frame bounds read after building both hierarchies -- must sync first,
            // otherwise Collider.bounds can return stale/unit-cube bounds.
            Physics.SyncTransforms();

            var robotCollider = robot.GetComponentInChildren<Collider>();
            var lowerBlock = GameObject.Find("LowerBlock");
            Assert.IsNotNull(lowerBlock, "LowerBlock not found in built course.");
            var blockBounds = lowerBlock.GetComponent<Collider>().bounds;

            float clearance = blockBounds.min.z - robotCollider.bounds.max.z;
            Assert.Greater(clearance, 0.1f,
                "Expected at least ~0.15 m clearance between the robot spawn footprint and LowerBlock, got " + clearance);
        }

        [UnityTest]
        public IEnumerator RobotSpawn_RestsOnFloorUnderGravity_WithoutFallingThrough()
        {
            course = CourseBuilder.BuildCourse();
            robot = RobotFactory.CreateRobot(CourseBuilder.RobotSpawnPosition + Vector3.up * 0.01f, CourseBuilder.RobotSpawnRotation);
            var rig = robot.GetComponent<RobotRig>();

            for (int i = 0; i < 100; i++) yield return new WaitForFixedUpdate();

            float finalY = rig.Body.transform.position.y;
            Assert.Greater(finalY, -0.05f, "Robot fell through the floor.");
            Assert.Less(finalY, 0.1f, "Robot is floating/bouncing well above the floor.");
        }

        [UnityTest]
        public IEnumerator RobotAtSpawn_DrivesForwardTowardStartLineWithoutImmediateCollision()
        {
            course = CourseBuilder.BuildCourse();
            robot = RobotFactory.CreateRobot(CourseBuilder.RobotSpawnPosition, CourseBuilder.RobotSpawnRotation);
            var rig = robot.GetComponent<RobotRig>();
            var drive = robot.AddComponent<Control.DifferentialDriveController>();
            drive.SetWheelSpeeds(0.3f, 0.3f);

            Vector3 startPos = rig.Body.transform.position;

            for (int i = 0; i < 100; i++) yield return new WaitForFixedUpdate();

            float traveled = Vector3.Distance(startPos, rig.Body.transform.position);
            Assert.Greater(traveled, 0.05f, "Robot should have driven forward from spawn without immediately colliding.");
        }
    }
}
