using UnityEngine;

namespace PIDReport.Trajectory
{
    // A straight-line move at a FIXED heading (inherited from wherever the previous
    // segment left off), with a minimum-jerk (quintic) speed profile sized so peak
    // acceleration never exceeds maxAccel. Camera-top acceleration equals chassis
    // acceleration for a straight (no rotation), so this directly respects the
    // 1.00 m/s^2 cap as long as maxAccel leaves tracking-error headroom under it.
    //
    // Heading is a constructor parameter, not derived from (start, targetPoint):
    // a real robot can't instantaneously snap heading at a segment boundary, so this
    // segment travels `length` along the incoming heading rather than aiming a new
    // heading at the next nominal waypoint -- it lands close to, but not necessarily
    // exactly on, that waypoint (see CourseTrajectoryPlanner).
    public class StraightSegment : TrajectorySegment
    {
        private readonly Vector3 start;
        private readonly Vector3 direction;
        private readonly float length;
        private readonly float headingRadians;

        public Vector3 EndPosition => start + direction * length;

        public StraightSegment(Vector3 start, float headingRadians, float length, float maxAccel)
        {
            this.start = start;
            this.headingRadians = headingRadians;
            this.length = length;
            direction = HeadingUtil.ToForward(headingRadians);
            Duration = MinimumJerkProfile.DurationForAccelCap(length, maxAccel);
        }

        public override TrajectoryState Evaluate(float t)
        {
            if (Duration <= 0f)
            {
                return new TrajectoryState { Position = start, HeadingRadians = headingRadians, Speed = 0f, AngularSpeed = 0f };
            }

            float tau = Mathf.Clamp01(t / Duration);
            float distance = length * MinimumJerkProfile.Position(tau);
            float speed = (length / Duration) * MinimumJerkProfile.Velocity(tau);

            return new TrajectoryState
            {
                Position = start + direction * distance,
                HeadingRadians = headingRadians,
                Speed = speed,
                AngularSpeed = 0f
            };
        }
    }
}
