using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace PIDReport.Tests
{
    // Regression test for the previous session's failure mode: the trajectory
    // planner believed the robot had moved while the Rigidbody never actually
    // translated. This isolates the most basic possible claim -- AddForce in
    // FixedUpdate on a non-kinematic Rigidbody produces real displacement --
    // before any of the real robot/controller stack is built on top of it.
    public class M1_BareRigidbodyMovementTests
    {
        [UnityTest]
        public IEnumerator AddForce_ProducesRealDisplacement_OverFixedSteps()
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            var rb = go.AddComponent<Rigidbody>();
            rb.mass = 1f;
            rb.useGravity = false; // isolate AddForce effect from gravity-driven fall

            Assert.IsFalse(rb.isKinematic,
                "Rigidbody must not be kinematic -- a kinematic body silently ignores AddForce.");

            Vector3 startPos = go.transform.position;

            var mover = go.AddComponent<TestForceMover>();
            mover.Force = new Vector3(50f, 0f, 0f);

            for (int i = 0; i < 30; i++)
            {
                yield return new WaitForFixedUpdate();
            }

            Vector3 endPos = go.transform.position;

            Assert.Greater(endPos.x - startPos.x, 0.01f,
                "Rigidbody did not move along the force axis after repeated AddForce calls " +
                "in FixedUpdate. Check Rigidbody.isKinematic and that AddForce is actually " +
                "being invoked, before building anything on top of this.");
            Assert.AreEqual(startPos.y, endPos.y, 0.001f,
                "Unexpected vertical displacement -- gravity should have been disabled for this isolated test.");

            Object.Destroy(go);
        }

        private class TestForceMover : MonoBehaviour
        {
            public Vector3 Force;
            private Rigidbody rb;
            void Awake() => rb = GetComponent<Rigidbody>();
            void FixedUpdate() => rb.AddForce(Force, ForceMode.Force);
        }
    }
}
