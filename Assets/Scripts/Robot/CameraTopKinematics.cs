using UnityEngine;

namespace PIDReport.Robot
{
    // Tracks the camera-top point's own horizontal-plane velocity/acceleration/jerk --
    // the point the 1.00 m/s^2 cap actually applies to, not the chassis. Deliberately
    // does not hand-derive a = a_ref + alpha x r + omega x (omega x r) term by term:
    // Rigidbody.GetPointVelocity(worldPoint) already folds the chassis's linear motion
    // and the rotational (centripetal + tangential) contribution together correctly for
    // whatever point you ask about, so finite-differencing that over time recovers the
    // full rigid-body point-acceleration result "for free" (calculus: d/dt[v_CoM + w x r]
    // = a_CoM + alpha x r + w x (w x r), since dr/dt = w x r for a point fixed to the body).
    [RequireComponent(typeof(RobotRig))]
    public class CameraTopKinematics : MonoBehaviour
    {
        private RobotRig rig;
        private Vector3 previousVelocity;
        private Vector3 previousAcceleration;
        private bool hasPreviousVelocity;
        private bool hasPreviousAcceleration;

        public Vector3 HorizontalVelocity { get; private set; }
        public Vector3 HorizontalAcceleration { get; private set; }
        public Vector3 HorizontalJerk { get; private set; }

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
        }
    }
}
