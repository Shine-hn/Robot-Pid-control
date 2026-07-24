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
        public float LinearGain = 300f;
        public float AngularGain = 3f;

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
        public float MaxAngularAcceleration = 15f;

        private RobotRig rig;
        private float wheelSpeedLeft;
        private float wheelSpeedRight;

        public float CommandedLinearSpeed => (wheelSpeedLeft + wheelSpeedRight) * 0.5f;
        public float CommandedAngularSpeed => (wheelSpeedLeft - wheelSpeedRight) / RobotRig.TrackWidth;

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

            Vector3 desiredVel = transform.forward * v;
            Vector3 currentVel = rb.linearVelocity;
            currentVel.y = 0f;
            Vector3 velError = desiredVel - currentVel;
            Vector3 force = Vector3.ClampMagnitude(velError * LinearGain, MaxAcceleration * rb.mass);
            rb.AddForce(force, ForceMode.Force);

            float currentAngVelY = rb.angularVelocity.y;
            float angError = omega - currentAngVelY;
            float maxTorque = MaxAngularAcceleration * rb.inertiaTensor.y;
            float torque = Mathf.Clamp(angError * AngularGain, -maxTorque, maxTorque);
            rb.AddTorque(Vector3.up * torque, ForceMode.Force);
        }
    }
}
