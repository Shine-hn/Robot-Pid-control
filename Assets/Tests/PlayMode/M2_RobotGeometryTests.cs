using System.Collections;
using System.Linq;
using NUnit.Framework;
using PIDReport.Robot;
using UnityEngine;
using UnityEngine.TestTools;

namespace PIDReport.Tests
{
    public class M2_RobotGeometryTests
    {
        private GameObject robot;

        [TearDown]
        public void TearDown()
        {
            if (robot != null) Object.Destroy(robot);
        }

        [Test]
        public void CreateRobot_ProducesExactlyOnePhysicsCollider()
        {
            robot = RobotFactory.CreateRobot(Vector3.zero, Quaternion.identity);
            var colliders = robot.GetComponentsInChildren<Collider>();

            Assert.AreEqual(1, colliders.Length,
                "Expected exactly one collider on the whole robot (the body). " +
                "Visual-only children (wheels, pole, camera head) must have their " +
                "auto-attached CreatePrimitive() colliders stripped.");
        }

        [Test]
        public void CreateRobot_RigidbodyIsConfiguredPerSpec()
        {
            robot = RobotFactory.CreateRobot(Vector3.zero, Quaternion.identity);
            var rig = robot.GetComponent<RobotRig>();
            var rb = rig.Body;

            Assert.IsFalse(rb.isKinematic,
                "Rigidbody must not be kinematic -- it would silently ignore all forces.");
            Assert.AreEqual(10f, rb.mass, 0.001f);
            Assert.AreEqual(new Vector3(0, 0.50f, 0), rb.centerOfMass);

            bool isContinuous =
                rb.collisionDetectionMode == CollisionDetectionMode.Continuous ||
                rb.collisionDetectionMode == CollisionDetectionMode.ContinuousDynamic ||
                rb.collisionDetectionMode == CollisionDetectionMode.ContinuousSpeculative;
            Assert.IsTrue(isContinuous,
                "Collision Detection must be set to a Continuous mode, was " + rb.collisionDetectionMode);
        }

        [Test]
        public void CreateRobot_CameraTopPointIsExactlyOneMeterAboveFloor()
        {
            robot = RobotFactory.CreateRobot(new Vector3(1f, 0f, 2f), Quaternion.identity);
            var rig = robot.GetComponent<RobotRig>();

            Assert.AreEqual(1.00f, rig.CameraTop.position.y, 0.0001f);
        }

        // Measures the AS-BUILT physics footprint rather than re-reading the constant the
        // factory was fed: the collider is a generated 32-sided convex hull scaled by the
        // body transform, so this is the assertion that actually proves 直径 0.30 m.
        [Test]
        public void CreateRobot_BodyDiameterMatchesSpec()
        {
            robot = RobotFactory.CreateRobot(Vector3.zero, Quaternion.identity);
            var collider = robot.GetComponentInChildren<Collider>();
            Bounds b = collider.bounds;

            // 32-sided polygon with a vertex on the +X axis => the AABB touches the true
            // 0.15 m radius exactly on X; Z is at worst a half-facet short (cos(pi/32)).
            Assert.AreEqual(0.30f, b.size.x, 0.005f, "Body diameter along X must be 0.30 m.");
            Assert.AreEqual(0.30f, b.size.z, 0.005f, "Body diameter along Z must be 0.30 m.");
        }

        [UnityTest]
        public IEnumerator RealRobotGeometry_StillMovesUnderAddForce()
        {
            robot = RobotFactory.CreateRobot(Vector3.zero, Quaternion.identity);
            var rig = robot.GetComponent<RobotRig>();
            var rb = rig.Body;
            rb.useGravity = false;

            Vector3 startPos = robot.transform.position;

            for (int i = 0; i < 60; i++)
            {
                rb.AddForce(Vector3.forward * 20f, ForceMode.Force);
                yield return new WaitForFixedUpdate();
            }

            Vector3 endPos = robot.transform.position;
            float forwardDisplacement = Vector3.Dot(endPos - startPos, Vector3.forward);

            Assert.Greater(forwardDisplacement, 0.01f,
                "The real robot rig did not move under AddForce the same way the bare " +
                "cube did in M1 -- check for a stray/duplicate collider fighting the floor, " +
                "or isKinematic being set somewhere during construction.");
        }
    }
}
