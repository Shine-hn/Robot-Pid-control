using UnityEngine;

namespace PIDReport.Trajectory
{
    // A pivot turn (信地旋回) about one stationary wheel: the chassis center sweeps a
    // fixed-radius arc (radius = TrackWidth/2) around the stationary wheel's world
    // position, using a minimum-jerk angular profile. Duration is sized so the peak
    // combined centripetal+tangential acceleration at the camera-top point (== chassis,
    // since the pole is mounted on the yaw axis -- see CameraTopKinematics) stays under
    // maxAccel. Pass radius=0 for a spin turn (超信地旋回, center fixed, zero-radius).
    public class TurnSegment : TrajectorySegment
    {
        private readonly Vector3 pivotPoint;
        private readonly float startHeadingRadians;
        private readonly float deltaHeadingRadians;
        private readonly float radius;
        private readonly Vector3 startRadialDir;

        public Vector3 EndPosition { get; }
        public float EndHeadingRadians => startHeadingRadians + deltaHeadingRadians;

        public TurnSegment(Vector3 centerStart, float startHeadingRadians, float deltaHeadingRadians,
            float radius, bool pivotOnRightWheel, float maxAccel)
        {
            this.startHeadingRadians = startHeadingRadians;
            this.deltaHeadingRadians = deltaHeadingRadians;
            this.radius = radius;

            if (radius > 1e-6f)
            {
                Vector3 right = HeadingUtil.ToRight(startHeadingRadians);
                Vector3 toPivot = (pivotOnRightWheel ? right : -right) * radius;
                pivotPoint = centerStart + toPivot;
                startRadialDir = -toPivot.normalized;
            }
            else
            {
                pivotPoint = centerStart;
                startRadialDir = Vector3.zero;
            }

            Duration = ComputeDuration(radius, deltaHeadingRadians, maxAccel);
            EndPosition = Evaluate(Duration).Position;
        }

        public override TrajectoryState Evaluate(float t)
        {
            if (Duration <= 0f)
            {
                Vector3 pos0 = radius > 1e-6f ? pivotPoint + startRadialDir * radius : pivotPoint;
                return new TrajectoryState { Position = pos0, HeadingRadians = startHeadingRadians, Speed = 0f, AngularSpeed = 0f };
            }

            float tau = Mathf.Clamp01(t / Duration);
            float angle = deltaHeadingRadians * MinimumJerkProfile.Position(tau);
            float angularSpeed = (deltaHeadingRadians / Duration) * MinimumJerkProfile.Velocity(tau);
            float heading = startHeadingRadians + angle;

            Vector3 position;
            float speed;
            if (radius > 1e-6f)
            {
                Vector3 radialDir = HeadingUtil.RotateY(startRadialDir, angle);
                position = pivotPoint + radialDir * radius;
                speed = angularSpeed * radius; // center always at fixed distance `radius` from the pivot wheel
            }
            else
            {
                position = pivotPoint; // spin turn: center fixed
                speed = 0f;
            }

            return new TrajectoryState { Position = position, HeadingRadians = heading, Speed = speed, AngularSpeed = angularSpeed };
        }

        // Both the centripetal (omega^2 * radius) and tangential (alpha * radius) terms
        // scale as 1/T^2, so the combined peak scales exactly as 1/T^2 too -- sample once
        // at a reference T=1s and solve for the T that brings that peak down to maxAccel,
        // rather than an iterative search.
        private static float ComputeDuration(float radius, float deltaHeading, float maxAccel)
        {
            if (Mathf.Abs(deltaHeading) < 1e-6f) return 0f;

            float peakAccelAtUnitDuration = 0f;
            const int samples = 200;
            for (int i = 0; i <= samples; i++)
            {
                float tau = i / (float)samples;
                float omega = deltaHeading * MinimumJerkProfile.Velocity(tau);      // at T=1s
                float alpha = deltaHeading * MinimumJerkProfile.Acceleration(tau);  // at T=1s
                float centripetal = omega * omega * radius;
                float tangential = Mathf.Abs(alpha) * radius;
                float total = Mathf.Sqrt(centripetal * centripetal + tangential * tangential);
                peakAccelAtUnitDuration = Mathf.Max(peakAccelAtUnitDuration, total);
            }

            if (peakAccelAtUnitDuration < 1e-9f) return 0f;
            return Mathf.Sqrt(peakAccelAtUnitDuration / maxAccel);
        }
    }
}
