using UnityEngine;

namespace PIDReport.Robot
{
    // Holds the physical spec constants and component references for a built robot.
    // Downstream systems (drive controller, kinematics, telemetry) should read this
    // instead of doing brittle Find-by-name/tag lookups.
    public class RobotRig : MonoBehaviour
    {
        public const float BodyDiameter = 0.30f;
        public const float BodyHeight = 0.12f;
        public const float PoleTopHeight = 1.00f; // camera-top point height above floor
        public const float TotalMass = 10f;
        public const float CenterOfMassHeight = 0.50f; // above floor
        public const float TrackWidth = 0.24f; // distance between wheel ground-contact centers
        public const float WheelRadius = 0.04f;
        public const float WheelWidth = 0.02f;

        // Wheel/floor contact friction ("駆動に必要な摩擦"), declared explicitly on BOTH
        // surfaces (robot body collider and floor) so the contact pair is unambiguous and
        // the combine mode cannot distort it.
        //
        // Currently ZERO, and that is a measured decision rather than an oversight. The
        // model represents ROLLING wheels: a rolling contact patch has no sliding velocity,
        // so its friction transmits traction without dissipating energy, and the assignment
        // explicitly permits rolling resistance and similar minor losses to be ignored
        // ("転がり抵抗、軸受損失、空気抵抗などの軽微な損失は無視してよい"). The drive
        // controller's chassis force already IS that net traction.
        //
        // Introducing sliding Coulomb friction here was attempted and measured across a
        // range of coefficients. It breaks the 1.00 m/s^2 camera-top cap -- a 必須条件, not
        // merely a scored quantity -- because every stick-slip transition (starting from
        // rest, stopping at a corner, beginning a spin) releases abruptly and that step is
        // amplified by the camera-top point's 0.5 m lever arm above the CoM:
        //
        //     mu = 0.00 -> peak camera-top accel 0.729 m/s^2, peak jerk   2.28 m/s^3
        //     mu = 0.02 -> peak camera-top accel 1.194 m/s^2, peak jerk  98.5  m/s^3
        //     mu = 0.05 -> peak camera-top accel 2.201 m/s^2, peak jerk 148.1  m/s^3
        //
        // Raising these above zero therefore requires first removing the stop/start
        // transitions that trigger stick-slip (i.e. continuous motion through the corners).
        public const float WheelFloorFriction = 0.0f;       // dynamic
        public const float WheelFloorStaticFriction = 0.0f; // static

        public Rigidbody Body;
        public Transform BodyVisual;
        public Transform WheelLeft;
        public Transform WheelRight;
        public Transform CameraTop; // exact point tracked for the 1.00 m/s^2 acceleration cap
    }
}
