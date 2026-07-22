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

        public Rigidbody Body;
        public Transform BodyVisual;
        public Transform WheelLeft;
        public Transform WheelRight;
        public Transform CameraTop; // exact point tracked for the 1.00 m/s^2 acceleration cap
    }
}
