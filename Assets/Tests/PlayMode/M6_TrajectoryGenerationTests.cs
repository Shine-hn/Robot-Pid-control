using NUnit.Framework;
using PIDReport.Course;
using PIDReport.Trajectory;
using UnityEngine;

namespace PIDReport.Tests
{
    public class M6_TrajectoryGenerationTests
    {
        [Test]
        public void CourseTrajectory_StartsAtSpawnAndEndsPastGoalLine()
        {
            var trajectory = CourseTrajectoryPlanner.BuildCourseTrajectory();

            var start = trajectory.Evaluate(0f);
            Assert.Less(Vector3.Distance(start.Position, CourseBuilder.RobotSpawnPosition), 0.01f);

            var end = trajectory.Evaluate(trajectory.TotalDuration);
            Assert.Less(end.Position.x, 1.80f, "Trajectory should end past the GoalLine (X < 1.80).");
            Assert.Greater(end.Position.x, 0.5f, "Trajectory should stop well within the course, not run off toward the wall.");
        }

        [Test]
        public void CourseTrajectory_NetHeadingChangeIsZero_ForARectangularLoop()
        {
            var trajectory = CourseTrajectoryPlanner.BuildCourseTrajectory();
            var start = trajectory.Evaluate(0f);
            var end = trajectory.Evaluate(trajectory.TotalDuration);

            float headingDelta = Mathf.DeltaAngle(start.HeadingRadians * Mathf.Rad2Deg, end.HeadingRadians * Mathf.Rad2Deg);
            Assert.Less(Mathf.Abs(headingDelta), 1f,
                "Two CW and two CCW 90-degree pivots should cancel out, ending at the same heading as the start.");
        }

        [Test]
        public void CourseTrajectory_IsPositionContinuousAcrossSegmentBoundaries()
        {
            var trajectory = CourseTrajectoryPlanner.BuildCourseTrajectory();
            float t = 0f;
            foreach (var segment in trajectory.Segments)
            {
                t += segment.Duration;
                if (t >= trajectory.TotalDuration - 1e-4f) continue;

                var justBefore = trajectory.Evaluate(t - 0.0005f);
                var justAfter = trajectory.Evaluate(t + 0.0005f);
                Assert.Less(Vector3.Distance(justBefore.Position, justAfter.Position), 0.01f,
                    "Trajectory position should not jump across a segment boundary at t=" + t);
            }
        }

        [Test]
        public void CourseTrajectory_ReferenceAccelerationNeverExceedsCap()
        {
            const float maxAccel = 0.6f; // matches a below-default cap so we can also check it's being respected, not just the default
            var trajectory = CourseTrajectoryPlanner.BuildCourseTrajectory(maxAccel);

            const float dt = 0.002f;
            Vector3 previousVelocity = VelocityAt(trajectory, 0f);
            float peakAccel = 0f;

            for (float t = dt; t <= trajectory.TotalDuration; t += dt)
            {
                Vector3 velocity = VelocityAt(trajectory, t);
                Vector3 accel = (velocity - previousVelocity) / dt;
                peakAccel = Mathf.Max(peakAccel, accel.magnitude);
                previousVelocity = velocity;
            }

            // Small numerical slack above the nominal cap for finite-difference sampling error.
            Assert.Less(peakAccel, maxAccel * 1.1f,
                "Reference trajectory's own kinematic acceleration exceeded its configured cap: " + peakAccel);
        }

        [Test]
        public void CourseTrajectory_DefaultMaxAccelLeavesHeadroomUnderHardCap()
        {
            Assert.Less(CourseTrajectoryPlanner.DefaultMaxAccel, 1.0f,
                "Default trajectory accel cap must leave headroom under the hard 1.00 m/s^2 requirement for closed-loop tracking error.");
        }

        private static Vector3 VelocityAt(RobotTrajectory trajectory, float t)
        {
            var state = trajectory.Evaluate(t);
            return HeadingUtil.ToForward(state.HeadingRadians) * state.Speed;
        }
    }
}
