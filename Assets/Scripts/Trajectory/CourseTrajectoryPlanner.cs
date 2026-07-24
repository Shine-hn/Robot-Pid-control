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
    // the pivot radius once the turn completes). Each following straight travels at
    // the FIXED heading the pivot ended at (a robot can't snap heading instantly) for
    // a distance equal to the projection of (nominal waypoint - actual start) onto
    // that heading -- landing close to, not exactly on, the nominal grid point.
    public static class CourseTrajectoryPlanner
    {
        // Kept below the 1.00 m/s^2 hard cap to leave headroom for closed-loop tracking
        // error on top of the reference trajectory's own kinematic acceleration.
        public const float DefaultMaxAccel = 0.7f;

        // Corners use 超信地旋回 (spin turns, zero radius) rather than 信地旋回 (pivot about
        // one wheel, radius = TrackWidth/2 = 0.12 m). A pivot swings the chassis CENTRE
        // 0.12 m sideways: entering the first corner on the corridor centreline at x = 0.30
        // it lands the centre at x = 0.18, putting the body's edge 0.030 m from WallWest --
        // a 3 cm margin that any tracking disturbance consumes (measured: it did, repeatedly,
        // once wheel/floor friction was introduced). A spin keeps the centre fixed, so the
        // robot stays on the corridor centreline with the full 0.15 m of clearance, and the
        // straights then land exactly on their nominal waypoints instead of 0.12 m off them.
        //
        // Spins are also cheaper against the acceleration budget: the camera-top point sits
        // on the yaw axis, so a spin contributes essentially NO camera-top acceleration,
        // whereas a pivot's centripetal + tangential terms did. Duration is matched to the
        // old pivots (~1.0 s for 90 degrees) so this costs no course time.
        // 信地旋回 remains fully supported by TurnSegment and covered by the M3 tests --
        // it is a required capability, just not the fastest way around this particular course.
        private const float SpinTurnRadius = 0f;

        public static RobotTrajectory BuildCourseTrajectory(float maxAccel = DefaultMaxAccel)
        {
            float radius = SpinTurnRadius;
            float headingNegX = HeadingUtil.FromForward(Vector3.left);

            var segments = new List<TrajectorySegment>();

            Vector3 center = Course.CourseBuilder.RobotSpawnPosition;
            float heading = headingNegX;

            var s1 = AddStraight(segments, center, heading, new Vector3(0.30f, 0f, 0.30f), maxAccel);
            center = s1.EndPosition;

            var t1 = new TurnSegment(center, heading, Mathf.PI / 2f, radius, pivotOnRightWheel: true, maxAccel);
            segments.Add(t1);
            center = t1.EndPosition;
            heading = t1.EndHeadingRadians;

            var s2 = AddStraight(segments, center, heading, new Vector3(0.30f, 0f, 1.50f), maxAccel);
            center = s2.EndPosition;

            var t2 = new TurnSegment(center, heading, Mathf.PI / 2f, radius, pivotOnRightWheel: true, maxAccel);
            segments.Add(t2);
            center = t2.EndPosition;
            heading = t2.EndHeadingRadians;

            var s3 = AddStraight(segments, center, heading, new Vector3(2.10f, 0f, 1.50f), maxAccel);
            center = s3.EndPosition;

            var t3 = new TurnSegment(center, heading, -Mathf.PI / 2f, radius, pivotOnRightWheel: false, maxAccel);
            segments.Add(t3);
            center = t3.EndPosition;
            heading = t3.EndHeadingRadians;

            var s4 = AddStraight(segments, center, heading, new Vector3(2.10f, 0f, 2.70f), maxAccel);
            center = s4.EndPosition;

            var t4 = new TurnSegment(center, heading, -Mathf.PI / 2f, radius, pivotOnRightWheel: false, maxAccel);
            segments.Add(t4);
            center = t4.EndPosition;
            heading = t4.EndHeadingRadians;

            // Final approach: continue past the GoalLine (X=1.80) with comfortable clearance.
            AddStraight(segments, center, heading, new Vector3(0.90f, 0f, 2.70f), maxAccel);

            return new RobotTrajectory(segments);
        }

        private static StraightSegment AddStraight(List<TrajectorySegment> segments, Vector3 start, float heading,
            Vector3 nominalTarget, float maxAccel)
        {
            float length = Vector3.Dot(nominalTarget - start, HeadingUtil.ToForward(heading));
            var segment = new StraightSegment(start, heading, length, maxAccel);
            segments.Add(segment);
            return segment;
        }
    }
}
