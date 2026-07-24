using UnityEngine;
using PIDReport.Robot;
using PIDReport.Trajectory;

namespace PIDReport.Control
{
    // Closed-loop trajectory tracking: a full state-feedback controller (the classic
    // unicycle/differential-drive tracking law -- see e.g. Siciliano et al., "Robotics:
    // Modelling, Planning and Control", trajectory tracking chapter) rather than plain
    // open-loop waypoint following. The reference trajectory's own feedforward speed and
    // turn rate are corrected each step by a feedback term on the robot-frame pose error,
    // and this is proven (via a Lyapunov argument on the error state) to drive tracking
    // error to zero given persistent excitation (v_r or omega_r nonzero).
    //
    // Error state, expressed in the ROBOT's own frame:
    //   e1 = forward-axis error   (how far behind/ahead of the reference point we are)
    //   e2 = right-axis error     (cross-track / lateral error)
    //   e3 = heading error        (wrapped to [-pi, pi])
    //
    // Control law:
    //   v     = v_r * cos(e3) + K1 * e1
    //   omega = omega_r + K2 * v_r * sinc(e3) * e2 + K3 * e3      (sinc(x) = sin(x)/x, sinc(0)=1)
    //
    // The resulting (v, omega) command is converted to wheel speeds and handed to
    // DifferentialDriveController, which is unchanged from M3 -- this class only decides
    // WHAT to command, not how forces get applied.
    [RequireComponent(typeof(RobotRig))]
    [RequireComponent(typeof(DifferentialDriveController))]
    public class TrajectoryTrackingController : MonoBehaviour
    {
        public float K1 = 2.0f;
        public float K2 = 8.0f;
        public float K3 = 3.0f;

        public float MaxCommandedSpeed = 2.0f;
        public float MaxCommandedAngularSpeed = 6.0f;

        private RobotRig rig;
        private DifferentialDriveController drive;
        private RobotTrajectory trajectory;

        public float ElapsedTime { get; private set; }
        public bool IsFinished => trajectory != null && ElapsedTime >= trajectory.TotalDuration;

        public float LongitudinalError { get; private set; }
        public float LateralError { get; private set; }
        public float HeadingError { get; private set; }
        public TrajectoryState ReferenceState { get; private set; }

        void Awake()
        {
            rig = GetComponent<RobotRig>();
            drive = GetComponent<DifferentialDriveController>();
        }

        public void SetTrajectory(RobotTrajectory newTrajectory)
        {
            trajectory = newTrajectory;
            ElapsedTime = 0f;
        }

        void FixedUpdate()
        {
            if (trajectory == null) return;

            var reference = trajectory.Evaluate(ElapsedTime);
            ReferenceState = reference;

            Vector3 currentPos = rig.Body.transform.position;
            float currentHeading = HeadingUtil.FromForward(rig.Body.transform.forward);

            Vector3 delta = reference.Position - currentPos;
            Vector3 forward = HeadingUtil.ToForward(currentHeading);
            Vector3 right = HeadingUtil.ToRight(currentHeading);

            float e1 = Vector3.Dot(delta, forward);
            float e2 = Vector3.Dot(delta, right);
            float e3 = Mathf.DeltaAngle(currentHeading * Mathf.Rad2Deg, reference.HeadingRadians * Mathf.Rad2Deg) * Mathf.Deg2Rad;

            LongitudinalError = e1;
            LateralError = e2;
            HeadingError = e3;

            float sinc = Mathf.Abs(e3) < 1e-4f ? 1f : Mathf.Sin(e3) / e3;

            float v = reference.Speed * Mathf.Cos(e3) + K1 * e1;
            float omega = reference.AngularSpeed + K2 * reference.Speed * sinc * e2 + K3 * e3;

            v = Mathf.Clamp(v, -MaxCommandedSpeed, MaxCommandedSpeed);
            omega = Mathf.Clamp(omega, -MaxCommandedAngularSpeed, MaxCommandedAngularSpeed);

            float wheelLeft = v + omega * RobotRig.TrackWidth * 0.5f;
            float wheelRight = v - omega * RobotRig.TrackWidth * 0.5f;
            drive.SetWheelSpeeds(wheelLeft, wheelRight);

            ElapsedTime += Time.fixedDeltaTime;
        }
    }
}
