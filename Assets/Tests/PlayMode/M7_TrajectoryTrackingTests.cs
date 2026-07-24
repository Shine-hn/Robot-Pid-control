using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using PIDReport.Control;
using PIDReport.Robot;
using PIDReport.Trajectory;
using UnityEngine;
using UnityEngine.TestTools;

namespace PIDReport.Tests
{
    public class M7_TrajectoryTrackingTests
    {
        private GameObject robot;

        private TrajectoryTrackingController Build(Vector3 spawnPos, float spawnHeadingRadians, RobotTrajectory trajectory)
        {
            Quaternion rotation = Quaternion.LookRotation(HeadingUtil.ToForward(spawnHeadingRadians), Vector3.up);
            robot = RobotFactory.CreateRobot(spawnPos, rotation);
            robot.GetComponent<RobotRig>().Body.useGravity = false;
            robot.AddComponent<DifferentialDriveController>();
            var tracker = robot.AddComponent<TrajectoryTrackingController>();
            tracker.SetTrajectory(trajectory);
            return tracker;
        }

        [TearDown]
        public void TearDown()
        {
            if (robot != null) Object.Destroy(robot);
        }

        [UnityTest]
        public IEnumerator TracksStraightSegment_SmallLateralAndHeadingError()
        {
            var trajectory = new RobotTrajectory(new List<TrajectorySegment>
            {
                new StraightSegment(Vector3.zero, 0f, 2f, 0.7f)
            });
            var tracker = Build(Vector3.zero, 0f, trajectory);

            float maxLateral = 0f, maxHeading = 0f;
            int steps = Mathf.CeilToInt(trajectory.TotalDuration / Time.fixedDeltaTime) + 20;
            for (int i = 0; i < steps; i++)
            {
                yield return new WaitForFixedUpdate();
                maxLateral = Mathf.Max(maxLateral, Mathf.Abs(tracker.LateralError));
                maxHeading = Mathf.Max(maxHeading, Mathf.Abs(tracker.HeadingError) * Mathf.Rad2Deg);
            }

            Assert.Less(maxLateral, 0.05f, "Lateral tracking error should stay small when starting exactly on the trajectory.");
            Assert.Less(maxHeading, 5f, "Heading tracking error should stay small (degrees) when starting exactly on the trajectory.");

            float finalDistance = Vector3.Distance(robot.transform.position, trajectory.Evaluate(trajectory.TotalDuration).Position);
            Assert.Less(finalDistance, 0.1f, "Robot should end up close to the trajectory's final point.");
        }

        [UnityTest]
        public IEnumerator ConvergesTowardTrajectory_WhenSpawnedWithLateralOffset()
        {
            var trajectory = new RobotTrajectory(new List<TrajectorySegment>
            {
                new StraightSegment(Vector3.zero, 0f, 2f, 0.7f)
            });
            // Spawn 0.15m to the side of the trajectory's start.
            var tracker = Build(new Vector3(0.15f, 0f, 0f), 0f, trajectory);

            float earlyLateral = 0f, lateLateral = 0f;
            int steps = Mathf.CeilToInt(trajectory.TotalDuration / Time.fixedDeltaTime) + 20;
            for (int i = 0; i < steps; i++)
            {
                yield return new WaitForFixedUpdate();
                if (i < 10) earlyLateral = Mathf.Max(earlyLateral, Mathf.Abs(tracker.LateralError));
                if (i >= steps - 10) lateLateral = Mathf.Max(lateLateral, Mathf.Abs(tracker.LateralError));
            }

            Assert.Greater(earlyLateral, 0.05f, "Should start with meaningful lateral error given the offset spawn.");
            Assert.Less(lateLateral, earlyLateral * 0.5f,
                "Closed-loop feedback should substantially reduce lateral error over the run, not just follow the reference open-loop.");
        }

        [UnityTest]
        public IEnumerator TracksTurnSegment_SmallHeadingError()
        {
            var trajectory = new RobotTrajectory(new List<TrajectorySegment>
            {
                new TurnSegment(Vector3.zero, 0f, Mathf.PI / 2f, RobotRig.TrackWidth * 0.5f, pivotOnRightWheel: true, maxAccel: 0.7f)
            });
            var tracker = Build(Vector3.zero, 0f, trajectory);

            float maxHeadingErrorDuringSettled = 0f;
            int steps = Mathf.CeilToInt(trajectory.TotalDuration / Time.fixedDeltaTime) + 20;
            for (int i = 0; i < steps; i++)
            {
                yield return new WaitForFixedUpdate();
                if (i > 10) // skip initial transient
                {
                    maxHeadingErrorDuringSettled = Mathf.Max(maxHeadingErrorDuringSettled, Mathf.Abs(tracker.HeadingError) * Mathf.Rad2Deg);
                }
            }

            Assert.Less(maxHeadingErrorDuringSettled, 10f, "Heading tracking error should stay reasonably small through a pivot turn.");

            float finalHeadingDeg = HeadingUtil.FromForward(robot.transform.forward) * Mathf.Rad2Deg;
            float expectedHeadingDeg = 90f;
            Assert.Less(Mathf.Abs(Mathf.DeltaAngle(finalHeadingDeg, expectedHeadingDeg)), 5f,
                "Robot should end the turn facing close to the commanded final heading.");
        }
    }
}
