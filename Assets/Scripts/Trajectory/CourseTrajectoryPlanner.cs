using System.Collections.Generic;
using UnityEngine;
using PIDReport.Robot;

namespace PIDReport.Trajectory
{
    // Builds the specific reference trajectory for this course's confirmed waypoint
    // sequence: spawn -> Pivot(0.30,0.30) -> (0.30,1.50) -> Pivot(0.30,1.50) ->
    // (2.10,1.50) -> Pivot(2.10,1.50) -> (2.10,2.70) -> Pivot(2.10,2.70) -> past GoalLine.
    // Each "Pivot" waypoint is the chassis center's position at the START of that pivot
    // (matching the maneuver's literal definition -- rotation about one stationary
    // wheel -- the center necessarily ends up offset from the nominal grid point by
    // the pivot radius once the turn completes); each following straight targets the
    // next nominal waypoint from wherever the pivot actually left the robot.
    public static class CourseTrajectoryPlanner
    {
        // Kept below the 1.00 m/s^2 hard cap to leave headroom for closed-loop tracking
        // error on top of the reference trajectory's own kinematic acceleration.
        public const float DefaultMaxAccel = 0.7f;

        public static RobotTrajectory BuildCourseTrajectory(float maxAccel = DefaultMaxAccel)
        {
            float radius = RobotRig.TrackWidth * 0.5f;
            float headingNegX = HeadingUtil.FromForward(Vector3.left);

            var segments = new List<TrajectorySegment>();

            Vector3 center = Course.CourseBuilder.RobotSpawnPosition;
            float heading = headingNegX;

            var s1 = new StraightSegment(center, new Vector3(0.30f, 0f, 0.30f), maxAccel);
            segments.Add(s1);
            center = s1.EndPosition;

            var t1 = new TurnSegment(center, heading, Mathf.PI / 2f, radius, pivotOnRightWheel: true, maxAccel);
            segments.Add(t1);
            center = t1.EndPosition;
            heading = t1.EndHeadingRadians;

            var s2 = new StraightSegment(center, new Vector3(0.30f, 0f, 1.50f), maxAccel);
            segments.Add(s2);
            center = s2.EndPosition;

            var t2 = new TurnSegment(center, heading, Mathf.PI / 2f, radius, pivotOnRightWheel: true, maxAccel);
            segments.Add(t2);
            center = t2.EndPosition;
            heading = t2.EndHeadingRadians;

            var s3 = new StraightSegment(center, new Vector3(2.10f, 0f, 1.50f), maxAccel);
            segments.Add(s3);
            center = s3.EndPosition;

            var t3 = new TurnSegment(center, heading, -Mathf.PI / 2f, radius, pivotOnRightWheel: false, maxAccel);
            segments.Add(t3);
            center = t3.EndPosition;
            heading = t3.EndHeadingRadians;

            var s4 = new StraightSegment(center, new Vector3(2.10f, 0f, 2.70f), maxAccel);
            segments.Add(s4);
            center = s4.EndPosition;

            var t4 = new TurnSegment(center, heading, -Mathf.PI / 2f, radius, pivotOnRightWheel: false, maxAccel);
            segments.Add(t4);
            center = t4.EndPosition;
            heading = t4.EndHeadingRadians;

            // Final approach: continue past the GoalLine (X=1.80) with comfortable clearance.
            var s5 = new StraightSegment(center, new Vector3(0.90f, 0f, 2.70f), maxAccel);
            segments.Add(s5);

            return new RobotTrajectory(segments);
        }
    }
}
