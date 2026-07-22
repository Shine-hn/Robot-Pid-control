using UnityEngine;

namespace PIDReport.Control
{
    // Interactive WASD test harness (legacy Input Manager) for manually confirming
    // the four required maneuvers before any trajectory/PID layer exists on top.
    // W/S: forward/backward. A/D: spin turn (超信地旋回, counter-rotate both wheels).
    // Q/E: pivot turn (信地旋回, one wheel stationary). No input: stop.
    [RequireComponent(typeof(DifferentialDriveController))]
    public class ManualDriveInput : MonoBehaviour
    {
        public float DriveSpeed = 0.3f;
        public float TurnSpeed = 0.3f;

        private DifferentialDriveController drive;

        void Awake()
        {
            drive = GetComponent<DifferentialDriveController>();
        }

        void Update()
        {
            float left = 0f;
            float right = 0f;

            if (Input.GetKey(KeyCode.W)) { left += DriveSpeed; right += DriveSpeed; }
            if (Input.GetKey(KeyCode.S)) { left -= DriveSpeed; right -= DriveSpeed; }

            if (Input.GetKey(KeyCode.A)) { left -= TurnSpeed; right += TurnSpeed; }
            if (Input.GetKey(KeyCode.D)) { left += TurnSpeed; right -= TurnSpeed; }

            if (Input.GetKey(KeyCode.Q)) { right += TurnSpeed; } // pivot about left wheel
            if (Input.GetKey(KeyCode.E)) { left += TurnSpeed; }  // pivot about right wheel

            drive.SetWheelSpeeds(left, right);
        }
    }
}
