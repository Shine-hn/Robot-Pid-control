using System.Collections;
using NUnit.Framework;
using PIDReport.Control;
using PIDReport.Robot;
using UnityEngine;
using UnityEngine.TestTools;

namespace PIDReport.Tests
{
    public class M3_DifferentialDriveTests
    {
        private GameObject robot;
        private DifferentialDriveController drive;
        private RobotRig rig;

        private void Build()
        {
            robot = RobotFactory.CreateRobot(Vector3.zero, Quaternion.identity);
            rig = robot.GetComponent<RobotRig>();
            rig.Body.useGravity = false;
            drive = robot.AddComponent<DifferentialDriveController>();
        }

        [TearDown]
        public void TearDown()
        {
            if (robot != null) Object.Destroy(robot);
        }

        [UnityTest]
        public IEnumerator Forward_MovesForwardWithoutRotating()
        {
            Build();
            drive.SetWheelSpeeds(0.3f, 0.3f);

            Quaternion startRot = robot.transform.rotation;
            Vector3 startPos = robot.transform.position;

            for (int i = 0; i < 120; i++) yield return new WaitForFixedUpdate();

            float forwardMoved = Vector3.Dot(robot.transform.position - startPos, Vector3.forward);
            float yawDrift = Quaternion.Angle(startRot, robot.transform.rotation);

            Assert.Greater(forwardMoved, 0.02f, "Equal wheel speeds should drive the robot forward.");
            Assert.Less(yawDrift, 2f, "Equal wheel speeds should not produce meaningful rotation.");
        }

        // Genuine PID: the integral term must reject a sustained disturbance that pure
        // proportional gain cannot. A constant 5 N drag opposing a 0.3 m/s command leaves a
        // P-only loop with a steady error of 5/LinearGain = 0.0167 m/s (final ~0.283 m/s);
        // the integrator supplies the 5 N itself and drives the steady speed to the command.
        [UnityTest]
        public IEnumerator PidIntegral_RejectsConstantDisturbance_PWouldLeaveSteadyError()
        {
            Build();
            drive.SetWheelSpeeds(0.3f, 0.3f);

            float finalSpeed = 0f;
            for (int i = 0; i < 400; i++)
            {
                rig.Body.AddForce(-robot.transform.forward * 5f, ForceMode.Force);
                yield return new WaitForFixedUpdate();
                finalSpeed = Vector3.Dot(rig.Body.linearVelocity, robot.transform.forward);
            }

            // P-only would settle at ~0.283 m/s; assert the integral closed most of that gap.
            Assert.Greater(finalSpeed, 0.295f,
                "PID integral should reject the constant disturbance and reach ~0.30 m/s; got " + finalSpeed);
        }

        // Anti-windup: under a disturbance so large the output stays saturated and the goal is
        // unreachable, the integrator must NOT grow without bound. Without back-calculation it
        // would accumulate every step; with it, it settles.
        [UnityTest]
        public IEnumerator PidAntiWindup_BoundsIntegralUnderSustainedSaturation()
        {
            Build();
            drive.SetWheelSpeeds(0.3f, 0.3f);

            float maxIntegral = 0f;
            for (int i = 0; i < 400; i++)
            {
                rig.Body.AddForce(-robot.transform.forward * 100f, ForceMode.Force); // 100 N >> 15 N clamp
                yield return new WaitForFixedUpdate();
                maxIntegral = Mathf.Max(maxIntegral, drive.LinearIntegral.magnitude);
            }

            // Un-clamped integration would reach error*dt*steps ~ many m/s over 400 steps and
            // keep climbing; anti-windup holds it to a small bound.
            Assert.Less(maxIntegral, 2f,
                "Anti-windup failed: integral wound up to " + maxIntegral + " under sustained saturation.");
        }

        // 後退: the assignment requires reverse as one of the five mandatory maneuvers
        // (前進/後退/停止/信地旋回/超信地旋回). Equal NEGATIVE wheel speeds must drive the
        // body backwards along its own -forward axis without yawing.
        [UnityTest]
        public IEnumerator Backward_MovesBackwardWithoutRotating()
        {
            Build();
            drive.SetWheelSpeeds(-0.3f, -0.3f);

            Quaternion startRot = robot.transform.rotation;
            Vector3 startPos = robot.transform.position;

            for (int i = 0; i < 120; i++) yield return new WaitForFixedUpdate();

            Vector3 delta = robot.transform.position - startPos;
            float forwardMoved = Vector3.Dot(delta, Vector3.forward);
            float yawDrift = Quaternion.Angle(startRot, robot.transform.rotation);

            Assert.Less(forwardMoved, -0.02f,
                "Equal negative wheel speeds should drive the robot backwards (負の前進方向).");
            Assert.Less(yawDrift, 2f,
                "Equal negative wheel speeds should not produce meaningful rotation.");
        }

        [UnityTest]
        public IEnumerator PivotTurn_LeftWheelStationary_StaysNearLeftWheelGroundPoint()
        {
            Build();
            drive.SetWheelSpeeds(0f, 0.3f); // right wheel drives, left wheel commanded stationary

            // Let the velocity/angular-velocity P-controller settle out of its startup
            // transient before measuring -- during the transient, linear and angular
            // tracking converge at different rates, so the "stationary" wheel briefly
            // drifts even though the steady-state pivot property holds.
            for (int i = 0; i < 90; i++) yield return new WaitForFixedUpdate();

            Vector3 leftWheelStart = rig.WheelLeft.position;
            Vector3 rightWheelStart = rig.WheelRight.position;

            for (int i = 0; i < 90; i++) yield return new WaitForFixedUpdate();

            float leftWheelDisplacement = Vector3.Distance(leftWheelStart, rig.WheelLeft.position);
            float rightWheelDisplacement = Vector3.Distance(rightWheelStart, rig.WheelRight.position);

            Assert.Greater(rightWheelDisplacement, 0.05f, "The driven wheel should actually have swept an arc.");
            Assert.Less(leftWheelDisplacement, rightWheelDisplacement * 0.3f,
                "Pivot turn should rotate about the stationary wheel -- its ground point should barely move " +
                "compared to the driven wheel, once the controller has settled.");
        }

        [UnityTest]
        public IEnumerator SpinTurn_CounterRotatingWheels_RotatesInPlace()
        {
            Build();
            Vector3 centerStart = robot.transform.position;
            Quaternion rotStart = robot.transform.rotation;

            drive.SetWheelSpeeds(-0.3f, 0.3f); // counter-rotate: zero-radius spin

            for (int i = 0; i < 150; i++) yield return new WaitForFixedUpdate();

            float centerDisplacement = Vector3.Distance(centerStart, robot.transform.position);
            float yawChange = Quaternion.Angle(rotStart, robot.transform.rotation);

            Assert.Greater(yawChange, 5f, "Spin turn should produce meaningful rotation.");
            Assert.Less(centerDisplacement, 0.05f,
                "Spin turn should be a zero-radius turn in place -- the chassis center should barely translate.");
        }

        [UnityTest]
        public IEnumerator Stop_BringsRobotToRest()
        {
            Build();
            drive.SetWheelSpeeds(0.3f, 0.3f);
            for (int i = 0; i < 60; i++) yield return new WaitForFixedUpdate();

            drive.SetWheelSpeeds(0f, 0f);
            for (int i = 0; i < 200; i++) yield return new WaitForFixedUpdate();

            float speed = rig.Body.linearVelocity.magnitude;
            Assert.Less(speed, 0.02f, "Commanding zero wheel speeds should bring the robot to rest.");
        }
    }
}
