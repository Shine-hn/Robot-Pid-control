using System.Collections;
using NUnit.Framework;
using PIDReport.Control;
using PIDReport.Robot;
using UnityEngine;
using UnityEngine.TestTools;

namespace PIDReport.Tests
{
    public class M5_CameraTopKinematicsTests
    {
        private GameObject robot;
        private RobotRig rig;
        private DifferentialDriveController drive;
        private CameraTopKinematics kinematics;

        private void Build()
        {
            robot = RobotFactory.CreateRobot(Vector3.zero, Quaternion.identity);
            rig = robot.GetComponent<RobotRig>();
            rig.Body.useGravity = false;
            drive = robot.AddComponent<DifferentialDriveController>();
            kinematics = robot.AddComponent<CameraTopKinematics>();
        }

        [TearDown]
        public void TearDown()
        {
            if (robot != null) Object.Destroy(robot);
        }

        [UnityTest]
        public IEnumerator SpinTurnInPlace_CameraTopAccelerationStaysNearZero()
        {
            Build();
            drive.SetWheelSpeeds(-0.3f, 0.3f); // pure spin: v=0, omega != 0

            float maxAccelMagnitude = 0f;
            for (int i = 0; i < 150; i++)
            {
                yield return new WaitForFixedUpdate();
                maxAccelMagnitude = Mathf.Max(maxAccelMagnitude, kinematics.HorizontalAcceleration.magnitude);
            }

            // The camera pole is mounted exactly on the robot's yaw axis (zero horizontal
            // offset from centerOfMass), so a pure spin -- rotation with no translation --
            // should produce ~zero horizontal acceleration at the camera-top point, even
            // though the chassis is very much angularly accelerating. This is the payoff
            // of that mounting choice: it eliminates the turn-induced acceleration penalty
            // for spin turns entirely.
            Assert.Less(maxAccelMagnitude, 0.5f,
                "Camera-top point should see near-zero horizontal acceleration during an in-place " +
                "spin turn when mounted on the yaw axis, but saw " + maxAccelMagnitude + " m/s^2.");
        }

        // Cross-validation of the two independent camera-top acceleration computations:
        // GetPointVelocity finite-difference vs the explicit EOM decomposition
        // a = a_CoM + alpha x r + omega x (omega x r). Under a pivot turn (translating AND
        // rotating) both the base and rotational terms are exercised; the two methods must
        // agree, confirming the explicit EOM is correct.
        [UnityTest]
        public IEnumerator ExplicitEomAcceleration_MatchesGetPointVelocityMethod()
        {
            Build();
            drive.SetWheelSpeeds(0f, 0.3f); // pivot: v != 0 and omega != 0

            float worstMismatch = 0f;
            float maxMagnitude = 0f;
            for (int i = 0; i < 150; i++)
            {
                yield return new WaitForFixedUpdate();
                // Skip the first couple of steps where each method's finite-difference state
                // is still priming (different warm-up lengths).
                if (i < 3) continue;
                Vector3 byPointVel = kinematics.HorizontalAcceleration;
                Vector3 byEom = kinematics.HorizontalAccelerationEOM;
                worstMismatch = Mathf.Max(worstMismatch, (byPointVel - byEom).magnitude);
                maxMagnitude = Mathf.Max(maxMagnitude, byPointVel.magnitude);
            }

            Assert.Greater(maxMagnitude, 0.05f,
                "Test is only meaningful if the camera-top actually accelerated; it did not.");
            // Both are finite differences at dt=0.02 s, so a small discretization gap is
            // expected; require them within 5% of the peak magnitude (plus a tiny floor).
            Assert.Less(worstMismatch, 0.05f * maxMagnitude + 0.02f,
                "Explicit EOM acceleration disagrees with the GetPointVelocity method by " +
                worstMismatch + " m/s^2 (peak " + maxMagnitude + ").");
        }

        [UnityTest]
        public IEnumerator PivotTurn_CameraTopAccelerationMatchesChassisAcceleration()
        {
            Build();
            drive.SetWheelSpeeds(0f, 0.3f); // pivot turn: v != 0 AND omega != 0 simultaneously

            // Skip the initial step-input transient: at t=0 the P-controller sees maximum
            // velocity error and produces a single large force spike, and comparing a
            // value sampled pre-physics-integration (inside CameraTopKinematics.FixedUpdate)
            // against one sampled post-integration (here, after WaitForFixedUpdate resumes)
            // means the two are one tick out of phase -- harmless once things settle, but
            // right at that sharp initial spike it makes an internally-consistent signal
            // look mismatched against itself by one frame. Measuring the settled average
            // instead of the instantaneous max is both robust to that and a more meaningful
            // test of "closely tracks" anyway.
            for (int i = 0; i < 10; i++) yield return new WaitForFixedUpdate();

            Vector3 previousChassisVel = rig.Body.linearVelocity;
            float totalDifference = 0f;
            float maxCameraTopAccel = 0f;
            int sampleCount = 140;

            for (int i = 0; i < sampleCount; i++)
            {
                yield return new WaitForFixedUpdate();

                Vector3 chassisVel = rig.Body.linearVelocity;
                chassisVel.y = 0f;
                Vector3 chassisAccel = (chassisVel - previousChassisVel) / Time.fixedDeltaTime;
                previousChassisVel = chassisVel;

                totalDifference += Vector3.Distance(chassisAccel, kinematics.HorizontalAcceleration);
                maxCameraTopAccel = Mathf.Max(maxCameraTopAccel, kinematics.HorizontalAcceleration.magnitude);
            }

            float averageDifference = totalDifference / sampleCount;

            Assert.Greater(maxCameraTopAccel, 0.05f,
                "A simultaneous translate+rotate maneuver should produce measurable camera-top acceleration.");
            // Since the pole sits exactly on the yaw axis, camera-top acceleration should track
            // the chassis (CoM) acceleration closely -- there is no extra offset-driven term.
            Assert.Less(averageDifference, 0.3f,
                "Camera-top acceleration should closely track chassis acceleration for an on-axis pole mount, " +
                "averaged difference was " + averageDifference + " m/s^2.");
        }

        [UnityTest]
        public IEnumerator ForwardAcceleration_ProducesForwardCameraTopAcceleration()
        {
            Build();
            drive.SetWheelSpeeds(0.3f, 0.3f);

            float maxForwardAccel = 0f;
            for (int i = 0; i < 30; i++) // sample the initial ramp-up, before it settles near zero
            {
                yield return new WaitForFixedUpdate();
                float forwardComponent = Vector3.Dot(kinematics.HorizontalAcceleration, robot.transform.forward);
                maxForwardAccel = Mathf.Max(maxForwardAccel, forwardComponent);
            }

            Assert.Greater(maxForwardAccel, 0.05f,
                "Accelerating forward should register positive forward acceleration at the camera-top point.");
        }
    }
}
