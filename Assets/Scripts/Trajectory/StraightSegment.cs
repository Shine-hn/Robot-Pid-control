using UnityEngine;

namespace PIDReport.Trajectory
{
    // A straight-line move at constant heading, with a minimum-jerk (quintic) speed
    // profile sized so peak acceleration never exceeds maxAccel. Camera-top acceleration
    // equals chassis acceleration for a straight (no rotation), so this directly respects
    // the 1.00 m/s^2 cap as long as maxAccel leaves tracking-error headroom under it.
    public class StraightSegment : TrajectorySegment
    {
        private readonly Vector3 start;
        private readonly Vector3 direction;
        private readonly float length;
        private readonly float headingRadians;

        public Vector3 EndPosition => start + direction * length;

        public StraightSegment(Vector3 start, Vector3 end, float maxAccel)
        {
            this.start = start;
            Vector3 delta = end - start;
            length = delta.magnitude;
            direction = length > 1e-6f ? delta / length : Vector3.forward;
            headingRadians = HeadingUtil.FromForward(direction);
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
