using UnityEngine;

namespace PIDReport.Trajectory
{
    // A straight-line segment that enters at one speed and leaves at another, accelerating
    // to a length-limited peak in between -- the piece that lets the course flow through
    // corners without stopping. (The original StraightSegment is always rest-to-rest, 0 ->
    // peak -> 0; it is kept unchanged for the M7 tests that construct it directly.)
    //
    // The speed profile is triangular: a minimum-jerk blend from entrySpeed up to a peak,
    // then a minimum-jerk blend down to exitSpeed, with no constant-speed cruise. Because
    // both halves are driven right up to maxAccel, the peak is whatever the length allows --
    // so a straight always uses its full acceleration budget without a separate speed cap to
    // tune. Solving d_accel(vp) + d_decel(vp) = length in closed form:
    //
    //     d_accel = 1.875 * (vp^2 - v0^2) / (2*a)      (see below for the 1.875)
    //     d_decel = 1.875 * (vp^2 - v1^2) / (2*a)
    //     => vp = sqrt( (2*a*L/1.875 + v0^2 + v1^2) / 2 )
    //
    // Each half is a minimum-jerk velocity blend v(tau) = va + (vb-va)*P(tau), with
    // P(tau) = 6tau^5 - 15tau^4 + 10tau^3 (MinimumJerkProfile.Position -- same quintic, used
    // here as the velocity S-curve). Its properties give everything else:
    //   * mean of P over [0,1] is 0.5, so the half covers distance T*(va+vb)/2 -> T = 2d/(va+vb);
    //   * P'(tau) peaks at 1.875 (tau=0.5), so peak accel = 1.875*(vb-va)/T = 1.875*(vb^2-va^2)/(2d);
    //   * P'(0) = P'(1) = 0, so acceleration is zero at both ends of each half -- the two
    //     halves join at zero acceleration, and the segment joins its neighbours (a constant-
    //     speed corner, or another straight) at zero longitudinal acceleration too. No jerk
    //     spike anywhere.
    public class VariableSpeedStraight : TrajectorySegment
    {
        private readonly Vector3 start;
        private readonly Vector3 direction;
        private readonly float headingRadians;

        private readonly float entrySpeed;
        private readonly float exitSpeed;
        private readonly float peakSpeed;
        private readonly float accelDuration;
        private readonly float accelDistance;
        private readonly float length;

        public Vector3 EndPosition => start + direction * length;
        public float EndHeadingRadians => headingRadians;
        public float PeakSpeed => peakSpeed;

        public VariableSpeedStraight(Vector3 start, float headingRadians, float length,
            float entrySpeed, float exitSpeed, float maxAccel)
        {
            this.start = start;
            this.headingRadians = headingRadians;
            this.length = length;
            this.entrySpeed = entrySpeed;
            this.exitSpeed = exitSpeed;
            direction = HeadingUtil.ToForward(headingRadians);

            if (length <= 1e-6f || maxAccel <= 0f)
            {
                peakSpeed = Mathf.Max(entrySpeed, exitSpeed);
                Duration = 0f;
                accelDuration = 0f;
                accelDistance = 0f;
                return;
            }

            float pf = MinimumJerkProfile.PeakVelocityFactor; // 1.875
            float vpSq = (2f * maxAccel * length / pf + entrySpeed * entrySpeed + exitSpeed * exitSpeed) * 0.5f;
            // Guard compares SQUARED speeds (vpSq is a v^2). Comparing it against a raw speed
            // would be a units mismatch that inflates the peak, shrinking the decel distance
            // and spiking the decel acceleration above maxAccel.
            float peakSq = Mathf.Max(vpSq, Mathf.Max(entrySpeed * entrySpeed, exitSpeed * exitSpeed));
            peakSpeed = Mathf.Sqrt(peakSq);

            accelDistance = pf * (peakSpeed * peakSpeed - entrySpeed * entrySpeed) / (2f * maxAccel);
            accelDistance = Mathf.Clamp(accelDistance, 0f, length);
            float decelDistance = length - accelDistance;

            accelDuration = SafeBlendDuration(accelDistance, entrySpeed, peakSpeed);
            float decelDuration = SafeBlendDuration(decelDistance, peakSpeed, exitSpeed);
            Duration = accelDuration + decelDuration;
        }

        private static float SafeBlendDuration(float distance, float va, float vb)
        {
            float mean = 0.5f * (va + vb);
            return mean > 1e-6f ? distance / mean : 0f;
        }

        public override TrajectoryState Evaluate(float t)
        {
            if (Duration <= 0f)
            {
                return new TrajectoryState
                {
                    Position = start,
                    HeadingRadians = headingRadians,
                    Speed = entrySpeed,
                    AngularSpeed = 0f
                };
            }

            float distance, speed;
            if (t <= accelDuration && accelDuration > 0f)
            {
                float tau = Mathf.Clamp01(t / accelDuration);
                distance = accelDuration * (entrySpeed * tau + (peakSpeed - entrySpeed) * BlendIntegral(tau));
                speed = entrySpeed + (peakSpeed - entrySpeed) * MinimumJerkProfile.Position(tau);
            }
            else
            {
                float decelDuration = Duration - accelDuration;
                float tau = decelDuration > 0f ? Mathf.Clamp01((t - accelDuration) / decelDuration) : 1f;
                float decelPart = decelDuration * (peakSpeed * tau + (exitSpeed - peakSpeed) * BlendIntegral(tau));
                distance = accelDistance + decelPart;
                speed = peakSpeed + (exitSpeed - peakSpeed) * MinimumJerkProfile.Position(tau);
            }

            return new TrajectoryState
            {
                Position = start + direction * distance,
                HeadingRadians = headingRadians,
                Speed = speed,
                AngularSpeed = 0f
            };
        }

        // Integral of the minimum-jerk velocity S-curve P(tau)=6tau^5-15tau^4+10tau^3:
        //   Q(tau) = tau^6 - 3tau^5 + 2.5tau^4,  with Q(1) = 0.5.
        // Used to turn a velocity blend into distance travelled.
        private static float BlendIntegral(float tau)
        {
            float t2 = tau * tau;
            float t4 = t2 * t2;
            return t4 * (t2 - 3f * tau + 2.5f);
        }
    }
}
