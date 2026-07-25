using UnityEngine;
using PIDReport.Robot;

namespace PIDReport.Control
{
    // Converts a commanded (left, right) wheel surface speed pair into chassis
    // linear/angular velocity via standard differential-drive kinematics, then
    // tracks that target with real AddForce/AddTorque -- never a Transform write.
    // Both are applied through the chassis (AddForce with no position acts through
    // the Rigidbody's centerOfMass; AddTorque is a pure couple) rather than at the
    // wheels' literal ground-contact points -- with centerOfMass overridden to
    // 0.50 m above a ~0.04 m-tall wheel plane, a force applied that far below the
    // CoM creates a large tipping moment and destabilizes the chassis. Driving
    // translation and rotation as two independent chassis-level channels avoids
    // that entirely.
    //
    // v      = (vL + vR) / 2
    // omega  = (vL - vR) / TrackWidth   (Unity is left-handed: +Y rotation turns
    //          the nose from forward toward right, which is exactly what a
    //          faster left wheel produces.)
    [RequireComponent(typeof(RobotRig))]
    public class DifferentialDriveController : MonoBehaviour
    {
        // Linear gain must be high enough that the linear-velocity loop's bandwidth
        // (Gain/mass) comfortably exceeds the turn rates this controller is asked to
        // track -- otherwise, since the target velocity direction is "current forward
        // * speed" and forward itself rotates at omega, a low-bandwidth loop perpetually
        // lags that rotating target instead of converging, producing a steady-state
        // tracking error (not just a slow transient) that shows up as drift during
        // pivot/spin turns. 300/10kg = 30/s bandwidth comfortably covers the turn
        // rates used here (~1-2 rad/s).
        // Gains are also constrained by discrete-time stability, not just continuous-time
        // convergence speed: for explicit-Euler integration of a first-order lag, the
        // per-step update is x[n+1] = x[n]*(1-k) + target*k where k = gain*fixedDeltaTime/inertia.
        // k > 2 makes the loop numerically oscillate/overshoot every step regardless of how
        // "correct" the gain looks in continuous time. With yaw inertia ~0.1125 kg*m^2 and
        // fixedDeltaTime 0.02s, AngularGain must stay well under 0.1125*2/0.02 = 11.25.
        // --- PID inner loop -------------------------------------------------------------
        // Each chassis channel (linear velocity, yaw rate) is tracked by a full PID, not just
        // proportional gain:
        //   * P (LinearGain / AngularGain) is the bulk of the response.
        //   * I (LinearKi / AngularKi) removes the small steady lag that pure P leaves while
        //     chasing a continuously accelerating / turning reference -- the error P alone
        //     needs to hold a nonzero output is integrated away instead.
        //   * D (LinearKd / AngularKd) adds damping so the I term cannot make the loop ring;
        //     taken on the MEASUREMENT (-d(velocity)/dt), not the error, so a step in the
        //     commanded setpoint does not produce a derivative "kick" (a jerk spike).
        // Integral gains are kept well into the overdamped range
        //   (zeta = Kp / (2*sqrt(Ki*inertia)): linear ~4.7, angular ~3.2)
        // so adding I never introduces oscillation -- oscillation would directly worsen the
        // scored jerk. Anti-windup is by back-calculation (see FixedUpdate): while the output
        // is saturated at the acceleration clamp, the integrator is bled back toward the
        // achievable value instead of winding up and overshooting on release.
        //
        // Gains are also kept modest against the force/torque budget so no single term can
        // dominate: with a ~15 N force clamp, the D term (Kd * measured accel) stays a few N
        // at the ~1 m/s^2 accelerations seen here, and the I term supplies only the small
        // residual the P term would otherwise hold an error to provide.
        //
        // The Kanayama trajectory-tracking law upstream (TrajectoryTrackingController) is a
        // separate state-feedback controller and is deliberately left untouched: this PID
        // regulates the chassis to whatever (v, omega) that law commands.
        public float LinearGain = 300f;   // linear P
        public float LinearKi = 100f;     // linear I
        public float LinearKd = 3f;       // linear D (on measurement)
        public float AngularGain = 3f;    // yaw P
        public float AngularKi = 2f;      // yaw I
        public float AngularKd = 0.05f;   // yaw D (on measurement)

        // Anti-windup by CONDITIONAL INTEGRATION plus an integral clamp. The integrator only
        // accumulates when the output is not saturated (or when the error is trying to pull
        // the output back out of saturation), and the accumulated integral term is capped so
        // it can never command more than the actuator can deliver. These caps are the maximum
        // FORCE / TORQUE the integral term alone is allowed to contribute (N and N*m); the
        // stored integral is those divided by Ki.
        public float LinearIntegralTermMax = 15f;   // = MaxAcceleration * mass
        public float AngularIntegralTermMax = 4.5f; // = MaxAngularAcceleration * Iyy

        // Real motor controllers always saturate; this one should too, both to guard
        // against a "zero moment point" tip-over (centerOfMass 0.5m above a ~0.15m-radius
        // support footprint tips at roughly 0.15*9.81/0.5 ~= 2.94 m/s^2) and as a backstop
        // against a runaway torque/force during an abnormally large transient error.
        //
        // These are NOT sized directly against the 1.00 m/s^2 camera-top cap, though --
        // that would be self-defeating: the M6 trajectory generator already sizes turn
        // duration so the REFERENCE profile's own peak angular acceleration (which, for
        // this course's 90-degree pivots, reaches ~5-6 rad/s^2) keeps combined camera-top
        // acceleration at exactly the configured cap. If this controller's own authority
        // is clamped below what the reference itself requires, it can't even follow the
        // safe profile it was given, let alone correct tracking error on top of it --
        // tried exactly that (0.6 m/s^2 / 3.0 rad/s^2) and the robot fell tens of degrees
        // behind the planned heading and drifted into a wall. The clamp needs headroom
        // *above* the reference's own peak requirement, wide enough that ordinary tracking
        // never saturates it, while still catching genuinely pathological errors.
        public float MaxAcceleration = 1.5f;

        // 15 rad/s^2 (= 15 * Iyy = 1.69 N*m) was enough while the ground contact was
        // frictionless, but is NOT enough to break yaw stiction now that the wheel/floor
        // pair declares real friction: the robot sat at the first corner and never rotated,
        // heading error saturating at 90 deg while the reference turned without it. Ideal
        // theory puts yaw stiction at only mu_s*N*(2/3)R = 0.59 N*m, but -- exactly as in
        // the measured linear sweep, where the real threshold was ~3x the ideal prediction
        // -- the 32-facet contact hull produces many simultaneous contact points and the
        // true figure lands just above the old 1.69 N*m ceiling.
        // Raising this is cheap and safe here: a spin turn rotates about the vertical axis
        // with the camera-top point ON that axis, so extra yaw authority adds no camera-top
        // acceleration and no tip-over exposure. Discrete-time stability is unaffected (it
        // depends on AngularGain*dt/Iyy = 0.53, not on the clamp).
        public float MaxAngularAcceleration = 40f;

        // Must match the PhysicsMaterial applied in RobotFactory. The wheel/floor contact
        // declares real friction (required by the assignment), but this model treats the
        // wheels as rolling rather than skidding -- a rolling contact patch has no sliding
        // velocity, so its friction transmits traction without dissipating energy. Unity
        // cannot express a rolling constraint on a plain collider, so the solver would
        // otherwise bill the chassis for skidding losses the assignment says to ignore
        // ("転がり抵抗...は無視してよい"). This feed-forward cancels exactly that drag.
        public float GroundFrictionCoefficient = RobotRig.WheelFloorFriction;
        public float GroundStaticFrictionCoefficient = RobotRig.WheelFloorStaticFriction;

        // Speed at which compensation finishes blending from the at-rest (stiction) case to
        // the rolling (kinetic) case. Coulomb friction is discontinuous through zero and its
        // direction is undefined at rest, so the two cases are blended rather than switched,
        // keeping the force continuous -- a step here would show up directly as jerk, which
        // is a scored quantity.
        public float FrictionCompensationFadeSpeed = 0.05f;

        // Same blend, for the yaw axis (rad/s).
        public float FrictionCompensationFadeSpinRate = 0.05f;

        // Effective moment arm of the ground contact patch for yaw friction. For a disc of
        // radius R under uniform pressure the resisting torque integrates to (2/3)*mu*N*R,
        // so the effective arm is (2/3)*R with R = the body radius.
        public float ContactPatchEffectiveRadius = (2f / 3f) * (RobotRig.BodyDiameter * 0.5f);

        private RobotRig rig;
        private float wheelSpeedLeft;
        private float wheelSpeedRight;

        // PID state.
        private Vector3 linearIntegral;
        private float angularIntegral;
        private Vector3 prevLinearVel;
        private float prevAngularVel;
        private bool hasPrevMeasurement;

        public float CommandedLinearSpeed => (wheelSpeedLeft + wheelSpeedRight) * 0.5f;
        public float CommandedAngularSpeed => (wheelSpeedLeft - wheelSpeedRight) / RobotRig.TrackWidth;

        // Exposed for the anti-windup regression test.
        public Vector3 LinearIntegral => linearIntegral;
        public float AngularIntegral => angularIntegral;

        void Awake()
        {
            rig = GetComponent<RobotRig>();
        }

        public void SetWheelSpeeds(float left, float right)
        {
            wheelSpeedLeft = left;
            wheelSpeedRight = right;
        }

        void FixedUpdate()
        {
            Rigidbody rb = rig.Body;

            float v = CommandedLinearSpeed;
            float omega = CommandedAngularSpeed;

            float dt = Time.fixedDeltaTime;

            Vector3 desiredVel = transform.forward * v;
            Vector3 currentVel = rb.linearVelocity;
            currentVel.y = 0f;
            Vector3 velError = desiredVel - currentVel;

            // PID: P on error, I on accumulated error, D on the MEASUREMENT (-d(vel)/dt) so a
            // setpoint step gives no derivative kick. Integral is used before it is updated.
            Vector3 linVelDeriv = hasPrevMeasurement ? (currentVel - prevLinearVel) / dt : Vector3.zero;
            Vector3 unclampedForce = LinearGain * velError + LinearKi * linearIntegral - LinearKd * linVelDeriv;
            float maxForce = MaxAcceleration * rb.mass;
            Vector3 force = Vector3.ClampMagnitude(unclampedForce, maxForce);

            // Conditional-integration anti-windup: accumulate only when not saturated, or when
            // the error opposes the saturation direction (so the integrator can unwind). Then
            // hard-clamp the integral term so it can never demand more than the actuator gives.
            bool linSaturated = unclampedForce.magnitude > maxForce;
            if (!linSaturated || Vector3.Dot(velError, unclampedForce) < 0f)
            {
                linearIntegral += velError * dt;
                linearIntegral = Vector3.ClampMagnitude(linearIntegral, LinearIntegralTermMax / LinearKi);
            }

            // Feed-forward cancellation of the solver's contact friction (see above).
            // Applied OUTSIDE the clamp because it is loss compensation, not commanded
            // control authority -- clamping it would silently eat into the acceleration
            // actually delivered.
            //
            // KINETIC drag only, along the actual sliding direction, faded in with speed.
            // Deliberately no static-friction term: static friction is a constraint force
            // that only opposes what is actually applied, so feeding forward the full
            // mu_s*N (11.8 N ~= 1.18 m/s^2) at rest over-drives the body and lurches it --
            // which walked the robot into WallWest in a corridor with only 0.15 m of
            // clearance. Stiction (mu_s*m*g = 11.8 N) sits below the 15 N drive clamp, so
            // the controller breaks it on its own authority without help.
            float speed = currentVel.magnitude;
            if (speed > 1e-4f)
            {
                float fade = Mathf.Clamp01(speed / FrictionCompensationFadeSpeed);
                float drag = GroundFrictionCoefficient * rb.mass * Mathf.Abs(Physics.gravity.y);
                force += (currentVel / speed) * drag * fade;
            }

            rb.AddForce(force, ForceMode.Force);

            float currentAngVelY = rb.angularVelocity.y;
            float angError = omega - currentAngVelY;
            float maxTorque = MaxAngularAcceleration * rb.inertiaTensor.y;

            // Same PID structure and conditional-integration anti-windup on the yaw channel.
            float angVelDeriv = hasPrevMeasurement ? (currentAngVelY - prevAngularVel) / dt : 0f;
            float unclampedTorque = AngularGain * angError + AngularKi * angularIntegral - AngularKd * angVelDeriv;
            float torque = Mathf.Clamp(unclampedTorque, -maxTorque, maxTorque);

            bool angSaturated = Mathf.Abs(unclampedTorque) > maxTorque;
            if (!angSaturated || angError * unclampedTorque < 0f)
            {
                angularIntegral += angError * dt;
                float angIntMax = AngularIntegralTermMax / AngularKi;
                angularIntegral = Mathf.Clamp(angularIntegral, -angIntMax, angIntMax);
            }

            prevLinearVel = currentVel;
            prevAngularVel = currentAngVelY;
            hasPrevMeasurement = true;

            // Yaw contact drag is deliberately NOT compensated. It was tried, and it is
            // actively harmful: a torque along sign(omega) is anti-damping, so any small
            // yaw rate gets reinforced rather than opposed. That turned into slow heading
            // wander, which in a 0.60 m corridor (0.15 m clearance per side) walked the
            // robot into WallWest partway around the course. Left alone, the same drag is
            // ordinary rotational damping that helps hold heading, and at mu = 0.05 it only
            // costs ~0.49 N*m out of ~1.69 N*m of available torque -- comfortably affordable.
            rb.AddTorque(Vector3.up * torque, ForceMode.Force);
        }
    }
}
