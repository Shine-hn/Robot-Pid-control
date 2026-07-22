using UnityEngine;

namespace PIDReport.Trajectory
{
    public struct TrajectoryState
    {
        public Vector3 Position;
        public float HeadingRadians;
        public float Speed;        // signed forward speed, m/s
        public float AngularSpeed; // rad/s, Unity convention (positive = clockwise from above)
    }

    public abstract class TrajectorySegment
    {
        public float Duration { get; protected set; }
        public abstract TrajectoryState Evaluate(float t);
    }
}
