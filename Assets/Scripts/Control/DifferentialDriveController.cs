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
            rb.AddForce(velError * LinearGain, ForceMode.Force);

            float currentAngVelY = rb.angularVelocity.y;
            float angError = omega - currentAngVelY;
            rb.AddTorque(Vector3.up * angError * AngularGain, ForceMode.Force);
        }
    }
}
