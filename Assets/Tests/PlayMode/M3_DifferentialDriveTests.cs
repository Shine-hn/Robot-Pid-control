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
