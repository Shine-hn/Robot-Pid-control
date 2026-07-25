using System.Collections.Generic;
using UnityEngine;
using PIDReport.Robot;

namespace PIDReport.Trajectory
{
    // Builds the reference trajectory for the confirmed S-crank course as a single
    // CONTINUOUS-MOTION path: straights joined to smooth (clothoid) corners, with speed
    // never returning to zero between the start and the goal.
    //
    // This replaces the earlier stop/turn/go plan (drive a straight to rest, rotate in
    // place, drive the next straight). That plan spent ~39% of the timed run at a crawl or
    // standstill purely because every corner required a full stop. Rounding each corner and
    // carrying speed through it removes those stops -- the single biggest lever on 走破時間.
    //
    // Geometry per corner: the corridor is only 0.60 m wide and the robot is 0.30 m across,
    // leaving 0.15 m of clearance. A smooth corner cannot be gentle (a large-radius arc
    // simply ploughs through the inner block), so the curvature is high and the corner speed
    // is correspondingly low (~0.2 m/s) -- but the corner is short, and the straights either
    // side stay fast, so the net is a large time saving. CornerCurvature is set from a
    // measured clearance sweep: kappa = 22 keeps every corner >= ~0.21 m from all obstacles
    // (0.06 m of margin beyond the robot radius for tracking error).
    //
    //   spawn(2.10,0.30) -> corner(0.30,0.30) -> corner(0.30,1.50) ->
    //   corner(2.10,1.50) -> corner(2.10,2.70) -> past GoalLine(0.90,2.70)
    //
    // The pivot / spin turn maneuvers (信地旋回 / 超信地旋回) remain implemented in
    // TurnSegment and covered by the M3 tests -- required capabilities, just not the fast
    // way around this particular course.
    public static class CourseTrajectoryPlanner
    {
        // Reference-trajectory acceleration budget. Applies to BOTH the straights'
        // longitudinal acceleration and the corners' lateral (centripetal) acceleration, so
        // the reference never asks for more than this in any direction. Kept below the
        // 1.00 m/s^2 hard cap to leave headroom for closed-loop tracking error on top.
        public const float DefaultMaxAccel = 0.78f;

        // Peak curvature of each clothoid corner (1/m). A measured clearance sweep with the
        // corners correctly placed on the corridor centrelines showed that the body gap is
        // fixed at 0.15 m by the centreline-to-wall distance for ANY kappa >= 5 -- the tight
        // corner never cuts closer to the inner block than the straights do. So clearance
        // does not push kappa up; the binding limit is only that the corner's setback must
        // fit inside the shortest (1.2 m) straight. Within that, a GENTLER corner is strictly
        // better on both scored axes: corner speed v = sqrt(maxAccel/kappa) rises as kappa
        // falls, and clothoid jerk (v^3 * kappa^2 = maxAccel^1.5 * sqrt(kappa)) falls with it.
        // kappa = 6 gives corner speed ~0.36 m/s with a 0.31 m setback -- comfortably inside
        // the 0.6 m half-straight -- so it is close to the fastest, smoothest feasible corner.
        public const float CornerCurvature = 6f;

        public static RobotTrajectory BuildCourseTrajectory(float maxAccel = DefaultMaxAccel)
        {
            float cornerSpeed = Mathf.Sqrt(maxAccel / CornerCurvature);

            float hNegX = HeadingUtil.FromForward(Vector3.left);   // -pi/2
            float hPosZ = HeadingUtil.FromForward(Vector3.forward); //  0
            float hPosX = HeadingUtil.FromForward(Vector3.right);   // +pi/2

            // Build the four corners first; each is fully determined by its centreline
            // intersection, the heading going in, and the signed 90-degree turn.
            var c1 = new SmoothCornerSegment(new Vector3(0.30f, 0f, 0.30f), hNegX, +Mathf.PI / 2f, CornerCurvature, cornerSpeed);
            var c2 = new SmoothCornerSegment(new Vector3(0.30f, 0f, 1.50f), hPosZ, +Mathf.PI / 2f, CornerCurvature, cornerSpeed);
            var c3 = new SmoothCornerSegment(new Vector3(2.10f, 0f, 1.50f), hPosX, -Mathf.PI / 2f, CornerCurvature, cornerSpeed);
            var c4 = new SmoothCornerSegment(new Vector3(2.10f, 0f, 2.70f), hPosZ, -Mathf.PI / 2f, CornerCurvature, cornerSpeed);

            Vector3 spawn = Course.CourseBuilder.RobotSpawnPosition;
            Vector3 finalEnd = new Vector3(0.90f, 0f, 2.70f);

            var segments = new List<TrajectorySegment>
            {
                // Flying start: accelerate from rest at spawn to corner speed by c1's entry.
                // The straight is 1.5 m before the StartLine, so the robot is already at full
                // speed when the clock starts.
                StraightBetween(spawn, c1.StartPosition, hNegX, 0f, cornerSpeed, maxAccel),
                c1,
                StraightBetween(c1.EndPosition, c2.StartPosition, hPosZ, cornerSpeed, cornerSpeed, maxAccel),
                c2,
                StraightBetween(c2.EndPosition, c3.StartPosition, hPosX, cornerSpeed, cornerSpeed, maxAccel),
                c3,
                StraightBetween(c3.EndPosition, c4.StartPosition, hPosZ, cornerSpeed, cornerSpeed, maxAccel),
                c4,
                // Final straight past the GoalLine (x=1.80), decelerating to rest at (0.90,2.70).
                StraightBetween(c4.EndPosition, finalEnd, hNegX, cornerSpeed, 0f, maxAccel),
            };

            return new RobotTrajectory(segments);
        }

        private static VariableSpeedStraight StraightBetween(Vector3 from, Vector3 to, float heading,
            float entrySpeed, float exitSpeed, float maxAccel)
        {
            // from and to are colinear along `heading` by construction (both on the same
            // corridor centreline), so the signed length is just the projection.
            float length = Vector3.Dot(to - from, HeadingUtil.ToForward(heading));
            return new VariableSpeedStraight(from, heading, length, entrySpeed, exitSpeed, maxAccel);
        }
    }
}
