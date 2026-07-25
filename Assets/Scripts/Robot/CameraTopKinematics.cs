using UnityEngine;

namespace PIDReport.Robot
{
    // Tracks the camera-top point's own horizontal-plane velocity/acceleration/jerk --
    // the point the 1.00 m/s^2 cap actually applies to, not the chassis.
    //
    // The primary path (HorizontalAcceleration) finite-differences
    // Rigidbody.GetPointVelocity(worldPoint), which already folds the chassis's linear
    // motion and the rotational (centripetal + tangential) contribution together correctly
    // for whatever point you ask about (calculus: d/dt[v_CoM + w x r] =
    // a_CoM + alpha x r + w x (w x r), since dr/dt = w x r for a point fixed to the body).
    //
    // AccelerationEOM computes that same rigid-body point-acceleration explicitly, term by
    // term, from the equation of motion:
    //
    //     a_point = a_CoM + alpha x r + omega x (omega x r)
    //               \____/   \______/   \______________/
    //                base    tangential   centripetal
    //
    // It exists as a cross-check on the built-in method (M5 asserts the two agree), and it
    // makes an otherwise-hidden result explicit: r points from the CoM straight up to the
    // camera on the yaw axis, so for a pure yaw (omega and r both vertical) BOTH rotational
    // terms vanish -- omega x r = 0. That is exactly why a spin-in-place adds no camera-top
    // acceleration, and why the camera-top horizontal acceleration essentially equals the
    // chassis's except for the small contribution of any body tilt (which tips r off
    // vertical and lets the centripetal term act).
    [RequireComponent(typeof(RobotRig))]
    public class CameraTopKinematics : MonoBehaviour
    {
        private RobotRig rig;
        private Vector3 previousVelocity;
        private Vector3 previousAcceleration;
        private bool hasPreviousVelocity;
        private bool hasPreviousAcceleration;

        private Vector3 previousBodyVelocity;
        private Vector3 previousAngularVelocity;
        private bool hasPreviousBodyState;

        public Vector3 HorizontalVelocity { get; private set; }
        public Vector3 HorizontalAcceleration { get; private set; }
        public Vector3 HorizontalJerk { get; private set; }

        // Same quantity as HorizontalAcceleration, computed via the explicit EOM
        // decomposition instead of GetPointVelocity -- for cross-validation.
        public Vector3 HorizontalAccelerationEOM { get; private set; }

        void Awake()
        {
            rig = GetComponent<RobotRig>();
        }

        void FixedUpdate()
        {
            Vector3 velocity = rig.Body.GetPointVelocity(rig.CameraTop.position);
            velocity.y = 0f; // horizontal-plane resultant only, gravity excluded
            HorizontalVelocity = velocity;

            if (hasPreviousVelocity)
            {
                Vector3 acceleration = (velocity - previousVelocity) / Time.fixedDeltaTime;
                HorizontalAcceleration = acceleration;

                if (hasPreviousAcceleration)
                {
                    HorizontalJerk = (acceleration - previousAcceleration) / Time.fixedDeltaTime;
                }
                previousAcceleration = acceleration;
                hasPreviousAcceleration = true;
            }

            previousVelocity = velocity;
            hasPreviousVelocity = true;

            ComputeEomAcceleration();
        }

        // Explicit a_point = a_CoM + alpha x r + omega x (omega x r), gravity excluded,
        // horizontal component only. a_CoM and alpha are finite-differenced (one-step lag,
        // matching the GetPointVelocity path), r and omega are taken at the current step.
        private void ComputeEomAcceleration()
        {
            var rb = rig.Body;
            Vector3 bodyVel = rb.linearVelocity;       // velocity of the centre of mass
            Vector3 omega = rb.angularVelocity;

            if (hasPreviousBodyState)
            {
                float dt = Time.fixedDeltaTime;
                Vector3 aCoM = (bodyVel - previousBodyVelocity) / dt;
                Vector3 alpha = (omega - previousAngularVelocity) / dt;
                Vector3 r = rig.CameraTop.position - rb.worldCenterOfMass;

                Vector3 tangential = Vector3.Cross(alpha, r);
                Vector3 centripetal = Vector3.Cross(omega, Vector3.Cross(omega, r));
                Vector3 aPoint = aCoM + tangential + centripetal;
                aPoint.y = 0f;
                HorizontalAccelerationEOM = aPoint;
            }

            previousBodyVelocity = bodyVel;
            previousAngularVelocity = omega;
            hasPreviousBodyState = true;
        }
    }
}
