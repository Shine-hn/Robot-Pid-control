using UnityEngine;

namespace PIDReport.Trajectory
{
    // A curvature-ramped 90-degree corner traversed at constant speed -- the continuous-
    // motion replacement for "decelerate to zero, turn in place, accelerate again".
    //
    // Why not a plain constant-radius arc: entering one at speed steps the curvature from 0
    // to 1/R instantly, so the centripetal term v^2/R appears within a single physics step.
    // At the speeds used here that is ~40 m/s^3 of jerk at every corner entry and exit --
    // a large regression on a scored quantity. Instead the curvature is ramped smoothly
    // (raised-cosine in arc length), which is the same idea as a clothoid/Euler-spiral
    // transition on a railway or road:
    //
    //     kappa(u) = kappa_max * (1 - cos(2*pi*u)) / 2,   u = s / L
    //
    // Integrating gives the total heading change kappa_max*L/2, so for a 90-degree corner
    // L = pi / kappa_max. Both the lateral acceleration and the jerk then stay bounded:
    //
    //     a_lateral(s) = v^2 * kappa(s)          -> peak v^2 * kappa_max
    //     jerk(s)      = v^3 * dkappa/ds         -> peak v^3 * kappa_max^2
    //
    // and both start and end at exactly zero, so the corner joins the adjoining straights
    // with no acceleration discontinuity at all.
    //
    // The path shape is obtained by numerically integrating the unit-speed curve, because a
    // clothoid has no closed form. Sampling is done once in the constructor and evaluated by
    // interpolation afterwards, so Evaluate() stays cheap enough for per-FixedUpdate use.
    public class SmoothCornerSegment : TrajectorySegment
    {
        private const int Samples = 256;

        private readonly Vector3[] positions = new Vector3[Samples + 1];
        private readonly float[] headings = new float[Samples + 1];
        private readonly float arcLength;
        private readonly float speed;
        private readonly float kappaMax;
        private readonly float deltaHeading;

        public Vector3 StartPosition => positions[0];
        public Vector3 EndPosition => positions[Samples];
        public float EndHeadingRadians => headings[Samples];
        public float ArcLength => arcLength;
        public float Speed => speed;

        // Distance from the corner's entry point to the intersection of the two corridor
        // centrelines. By symmetry the exit sits the same distance past it on the outgoing
        // centreline, so the planner can place the corner purely from this number.
        public float EntryOffset { get; private set; }

        public float PeakLateralAcceleration => speed * speed * kappaMax;
        public float PeakJerk => speed * speed * speed * kappaMax * kappaMax;

        // Minimum distance from the swept path to an arbitrary world point -- used by the
        // planner/tests to verify corridor clearance against the inner block corner.
        public float MinimumDistanceTo(Vector3 point)
        {
            float best = float.MaxValue;
            for (int i = 0; i <= Samples; i++)
            {
                Vector3 d = positions[i] - point;
                d.y = 0f;
                best = Mathf.Min(best, d.magnitude);
            }
            return best;
        }

        /// <param name="intersection">Where the two corridor centrelines cross.</param>
        /// <param name="approachHeading">Heading travelling INTO the corner.</param>
        /// <param name="deltaHeading">+/- pi/2.</param>
        /// <param name="kappaMax">Peak curvature (1/m); tightest point of the turn.</param>
        /// <param name="speed">Constant speed held through the corner.</param>
        public SmoothCornerSegment(Vector3 intersection, float approachHeading, float deltaHeading,
            float kappaMax, float speed)
        {
            this.kappaMax = kappaMax;
            this.speed = speed;
            this.deltaHeading = deltaHeading;

            arcLength = Mathf.PI / kappaMax * (Mathf.Abs(deltaHeading) / (Mathf.PI * 0.5f));
            Duration = arcLength / speed;

            // Integrate the curve directly in WORLD heading, starting at approachHeading,
            // from an arbitrary origin. (An earlier version integrated in a local +X frame
            // and rotated into world, but Unity's RotateY maps local +X to the world RIGHT
            // vector, not the forward/approach direction -- which silently misplaced the
            // exit of the -90 degree corners. Integrating in world heading avoids the whole
            // local->world mapping.)
            float ds = arcLength / Samples;
            float heading = approachHeading;
            Vector3 pos = Vector3.zero;
            positions[0] = pos;
            headings[0] = heading;
            for (int i = 1; i <= Samples; i++)
            {
                float uMid = (i - 0.5f) / Samples;
                float kappaMid = CurvatureShape(uMid) * kappaMax * Mathf.Sign(deltaHeading);
                float headingMid = heading + kappaMid * ds * 0.5f;
                pos += HeadingUtil.ToForward(headingMid) * ds;
                heading += kappaMid * ds;
                positions[i] = pos;
                headings[i] = heading;
            }

            // Place the (shape-fixed) curve so its entry lands on the incoming centreline and
            // its exit on the outgoing one, both through the intersection. For a symmetric
            // 90-degree corner the net displacement D = positions[N]-positions[0] is parallel
            // to (approachDir + exitDir), with D = d*(approachDir + exitDir); solving for the
            // setback d places entry at intersection - approachDir*d and exit at
            // intersection + exitDir*d.
            Vector3 approachDir = HeadingUtil.ToForward(approachHeading);
            Vector3 exitDir = HeadingUtil.ToForward(approachHeading + deltaHeading);
            Vector3 sum = approachDir + exitDir;
            Vector3 net = positions[Samples] - positions[0];
            EntryOffset = Vector3.Dot(net, sum) / Vector3.Dot(sum, sum);

            Vector3 entryWorld = intersection - approachDir * EntryOffset;
            Vector3 offset = entryWorld - positions[0];
            for (int i = 0; i <= Samples; i++) positions[i] += offset;
        }

        // Raised cosine over u in [0,1]: zero at both ends (so curvature, and therefore
        // lateral acceleration, joins the neighbouring straights continuously) and unit
        // mean, which is what makes the total heading change exactly kappa_max*L/2.
        private static float CurvatureShape(float u)
        {
            return (1f - Mathf.Cos(2f * Mathf.PI * u)) * 0.5f;
        }

        public override TrajectoryState Evaluate(float t)
        {
            if (Duration <= 0f)
            {
                return new TrajectoryState
                {
                    Position = positions[0],
                    HeadingRadians = headings[0],
                    Speed = 0f,
                    AngularSpeed = 0f
                };
            }

            float tau = Mathf.Clamp01(t / Duration);
            float fi = tau * Samples;
            int i = Mathf.Clamp(Mathf.FloorToInt(fi), 0, Samples - 1);
            float frac = fi - i;

            Vector3 pos = Vector3.Lerp(positions[i], positions[i + 1], frac);
            float heading = Mathf.Lerp(headings[i], headings[i + 1], frac);
            float kappa = CurvatureShape(tau) * kappaMax * Mathf.Sign(deltaHeading);

            return new TrajectoryState
            {
                Position = pos,
                HeadingRadians = heading,
                Speed = speed,
                AngularSpeed = speed * kappa
            };
        }
    }
}
