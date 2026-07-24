using UnityEngine;
using PIDReport.Robot;
using PIDReport.Course;
using PIDReport.Control;

namespace PIDReport.Race
{
    // Owns both halves of brief Section 1's requirements that aren't pure physics:
    // timing (StartLine touch -> GoalLine full clearance) and run invalidation (wall
    // contact, off-course passthrough backstop, tip-over). Both are tracked
    // independently and left as public state for downstream consumers (telemetry,
    // regression tests, the final report) to interpret -- e.g. "ignore FinishTime if
    // IsInvalidated" is a caller decision, not something this class enforces itself.
    [RequireComponent(typeof(RobotRig))]
    public class RaceManager : MonoBehaviour
    {
        // A little outside the walls' own outer face (not just their inner/drivable
        // face) so this only fires on genuine passthrough, not on ordinary wall contact
        // (which OnCollisionEnter already reports on its own).
        public float MinX = -0.1f, MaxX = 2.5f, MinZ = -0.1f, MaxZ = 3.1f;
        public float MaxTipAngleDegrees = 30f;

        public bool RaceStarted { get; private set; }
        public bool RaceFinished { get; private set; }
        public float StartTime { get; private set; }
        public float FinishTime { get; private set; }

        public bool IsInvalidated { get; private set; }
        public string InvalidationReason { get; private set; }

        public float ElapsedSimTime { get; private set; }
        public float CourseTime => RaceFinished ? FinishTime - StartTime
            : (RaceStarted ? ElapsedSimTime - StartTime : 0f);

        private RobotRig rig;
        private DifferentialDriveController drive;

        void Awake()
        {
            rig = GetComponent<RobotRig>();
            drive = GetComponent<DifferentialDriveController>();

            var startLine = GameObject.FindGameObjectWithTag("StartLine");
            if (startLine != null)
            {
                var trigger = startLine.GetComponent<LineTrigger>();
                if (trigger != null) trigger.Entered += OnStartLineEntered;
            }

            var goalLine = GameObject.FindGameObjectWithTag("GoalLine");
            if (goalLine != null)
            {
                var trigger = goalLine.GetComponent<LineTrigger>();
                if (trigger != null) trigger.Exited += OnGoalLineExited;
            }
        }

        void FixedUpdate()
        {
            ElapsedSimTime += Time.fixedDeltaTime;

            if (IsInvalidated) return;

            Vector3 pos = rig.Body.transform.position;
            if (pos.x < MinX || pos.x > MaxX || pos.z < MinZ || pos.z > MaxZ)
            {
                Invalidate("Robot moved outside the drivable course area at " + pos);
                return;
            }

            float tiltAngle = Vector3.Angle(rig.Body.transform.up, Vector3.up);
            if (tiltAngle > MaxTipAngleDegrees)
            {
                Invalidate("Robot tipped over (tilt=" + tiltAngle.ToString("F1") + " deg)");
            }
        }

        // Unity dispatches collision callbacks to the GameObject that owns the
        // Rigidbody, not to the child GameObject that owns the Collider -- so this
        // belongs directly on RaceManager (root, alongside the Rigidbody/RobotRig),
        // not on a script sitting next to the collider on the "Body" child.
        void OnCollisionEnter(Collision collision)
        {
            if (collision.collider.CompareTag("Wall"))
            {
                Invalidate("Robot contacted a wall (" + collision.collider.name + ")");
            }
        }

        private void OnStartLineEntered(Collider other)
        {
            if (!RaceStarted)
            {
                RaceStarted = true;
                StartTime = ElapsedSimTime;
            }
        }

        private void OnGoalLineExited(Collider other)
        {
            if (RaceStarted && !RaceFinished)
            {
                RaceFinished = true;
                FinishTime = ElapsedSimTime;
            }
        }

        private void Invalidate(string reason)
        {
            if (IsInvalidated) return;
            IsInvalidated = true;
            InvalidationReason = reason;

            // With no drive-wheel torque limit modeled (per spec), a controller with no
            // awareness of collisions will keep commanding full force into an obstacle
            // indefinitely once blocked. Sustained pushing at an off-axis contact point,
            // combined with this robot's very high centerOfMass, can accumulate enough
            // torque to tip it over well after the initial (otherwise survivable) contact.
            // Once a run is invalidated there is nothing left to gain by continuing to
            // drive, so cut the commanded wheel speeds.
            drive?.SetWheelSpeeds(0f, 0f);
        }
    }
}
