using UnityEngine;

namespace PIDReport.Trajectory
{
    // Quintic minimum-jerk point-to-point profile (Flash & Hogan): position, velocity
    // and acceleration are all zero at both endpoints (tau=0 and tau=1), so chaining
    // segments never produces an acceleration discontinuity, and for a fixed move
    // duration this specific polynomial minimizes integrated squared jerk -- a smooth,
    // well-founded S-curve rather than an ad-hoc easing curve.
    public static class MinimumJerkProfile
    {
        // Peak of Velocity(tau) over [0,1], occurring at tau=0.5.
        public const float PeakVelocityFactor = 1.875f;

        // Peak of |Acceleration(tau)| over [0,1], occurring at tau = (3 -/+ sqrt(3)) / 6.
        public const float PeakAccelerationFactor = 5.7735027f;

        public static float Position(float tau)
        {
            float t2 = tau * tau;
            float t3 = t2 * tau;
            return 6f * t3 * t2 - 15f * t2 * t2 + 10f * t3;
        }

        public static float Velocity(float tau)
        {
            float t2 = tau * tau;
            float t3 = t2 * tau;
            return 30f * t2 * t2 - 60f * t3 + 30f * t2;
        }

        public static float Acceleration(float tau)
        {
            float t2 = tau * tau;
            return 120f * t2 * tau - 180f * t2 + 60f * tau;
        }

        // Duration T such that a straight-line move of `distance` over time T keeps
        // peak acceleration at exactly `maxAccel` (peak scales as distance/T^2).
        public static float DurationForAccelCap(float distance, float maxAccel)
        {
            float d = Mathf.Abs(distance);
            if (d < 1e-6f || maxAccel <= 0f) return 0f;
            return Mathf.Sqrt(PeakAccelerationFactor * d / maxAccel);
        }
    }
}
