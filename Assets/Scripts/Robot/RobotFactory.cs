using UnityEngine;

namespace PIDReport.Robot
{
    // Builds the robot GameObject hierarchy at runtime (never authored as a hand-edited
    // prefab, so there is no risk of stray/duplicate colliders sneaking in via the Editor).
    // Root transform sits at floor level (local Y=0 = floor), matching the spec's habit of
    // measuring heights "above the floor".
    public static class RobotFactory
    {
        public static GameObject CreateRobot(Vector3 position, Quaternion rotation)
        {
            var root = new GameObject("Robot");
            root.tag = "Robot";
            root.transform.SetPositionAndRotation(position, rotation);

            var body = CreateVisualOnly(PrimitiveType.Cylinder, "Body", root.transform);
            body.transform.localPosition = new Vector3(0, RobotRig.BodyHeight * 0.5f, 0);
            body.transform.localScale = new Vector3(RobotRig.BodyDiameter, RobotRig.BodyHeight * 0.5f, RobotRig.BodyDiameter);

            // Single physics collider for the whole robot: a convex hull of the body
            // cylinder. A capsule collider (Unity's default for the Cylinder primitive)
            // would be inaccurate for this squat, flat-puck shape.
            var meshCollider = body.AddComponent<MeshCollider>();
            meshCollider.sharedMesh = body.GetComponent<MeshFilter>().sharedMesh;
            meshCollider.convex = true;

            // The drive controller's chassis force already IS the abstracted net
            // wheel-traction force ("sufficient friction for traction is assumed" --
            // brief). Unity's default PhysicsMaterial has nonzero friction, which would
            // additionally treat the chassis as a block sliding against the floor and
            // resist that force on top of it -- effectively double-counting resistance
            // the brief says to ignore ("ignore rolling resistance"), and strong enough
            // (with this robot's weight) to fully cancel a safely torque-limited drive
            // force. Zero it out so the floor/walls only constrain motion, never resist it.
            meshCollider.material = new PhysicsMaterial("Frictionless")
            {
                dynamicFriction = 0f,
                staticFriction = 0f,
                bounciness = 0f,
                frictionCombine = PhysicsMaterialCombine.Minimum,
                bounceCombine = PhysicsMaterialCombine.Minimum
            };

            var wheelLeft = CreateWheel("WheelLeft", root.transform, -RobotRig.TrackWidth * 0.5f);
            var wheelRight = CreateWheel("WheelRight", root.transform, RobotRig.TrackWidth * 0.5f);

            float poleHeight = RobotRig.PoleTopHeight - RobotRig.BodyHeight;
            var pole = CreateVisualOnly(PrimitiveType.Cylinder, "Pole", root.transform);
            pole.transform.localPosition = new Vector3(0, RobotRig.BodyHeight + poleHeight * 0.5f, 0);
            pole.transform.localScale = new Vector3(0.03f, poleHeight * 0.5f, 0.03f);

            var cameraHead = CreateVisualOnly(PrimitiveType.Cube, "CameraHead", root.transform);
            cameraHead.transform.localPosition = new Vector3(0, RobotRig.PoleTopHeight, 0);
            cameraHead.transform.localScale = new Vector3(0.06f, 0.06f, 0.06f);

            var cameraTop = new GameObject("CameraTopPoint");
            cameraTop.transform.SetParent(root.transform, false);
            cameraTop.transform.localPosition = new Vector3(0, RobotRig.PoleTopHeight, 0);

            var rb = root.AddComponent<Rigidbody>();
            rb.mass = RobotRig.TotalMass;
            // ContinuousDynamic (avoids tunneling through OTHER fast-moving dynamic
            // rigidbodies too) was tried first, but that combined with a convex
            // MeshCollider silently suppressed OnCollisionEnter/Stay dispatch entirely
            // while still resolving the contact physically -- a known-flaky combination.
            // The only tunneling concern here is a single dynamic robot against static
            // course geometry, which plain Continuous mode (sweeps against static
            // colliders only) covers correctly and reliably delivers collision events for.
            rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.isKinematic = false;

            // automaticCenterOfMass/automaticInertiaTensor default to true, which makes
            // Unity silently recompute both from collider geometry (around ITS OWN
            // centroid) whenever the physics scene re-evaluates the body -- discarding
            // any manual override made below. Both must be turned off first, otherwise
            // the manual values only "stick" for an instantaneous read right after
            // assignment and are gone again before the first physics step.
            rb.automaticCenterOfMass = false;
            rb.automaticInertiaTensor = false;

            rb.centerOfMass = new Vector3(0, RobotRig.CenterOfMassHeight, 0);

            // With CoM pushed 0.5 m above a ~0.15 m-radius, 0.12 m-tall body collider,
            // Unity's (now-disabled) auto inertia calculation would otherwise be wildly
            // inconsistent with the robot's actual (assumed) mass distribution. Set it
            // explicitly instead, modeling the robot as a uniform solid cylinder spanning
            // floor to camera-top (radius = body radius, height = pole height) -- consistent
            // with the given CoM height, since a uniform cylinder's centroid sits at half
            // its height.
            float r = RobotRig.BodyDiameter * 0.5f;
            float h = RobotRig.PoleTopHeight;
            float m = RobotRig.TotalMass;
            float yawInertia = 0.5f * m * r * r;
            float tiltInertia = (1f / 12f) * m * (3f * r * r + h * h);
            rb.inertiaTensor = new Vector3(tiltInertia, yawInertia, tiltInertia);
            rb.inertiaTensorRotation = Quaternion.identity;

            var rig = root.AddComponent<RobotRig>();
            rig.Body = rb;
            rig.BodyVisual = body.transform;
            rig.WheelLeft = wheelLeft.transform;
            rig.WheelRight = wheelRight.transform;
            rig.CameraTop = cameraTop.transform;

            return root;
        }

        private static GameObject CreateWheel(string name, Transform parent, float localX)
        {
            var wheel = CreateVisualOnly(PrimitiveType.Cylinder, name, parent);
            wheel.transform.localPosition = new Vector3(localX, RobotRig.WheelRadius, 0);
            wheel.transform.localRotation = Quaternion.Euler(0, 0, 90); // axle along local X
            wheel.transform.localScale = new Vector3(RobotRig.WheelRadius * 2f, RobotRig.WheelWidth * 0.5f, RobotRig.WheelRadius * 2f);
            return wheel;
        }

        // CreatePrimitive() auto-attaches a collider to every object. For visual-only
        // children (wheels, pole, camera head) that collider must be stripped immediately,
        // otherwise it fights the floor/other colliders every physics step.
        private static GameObject CreateVisualOnly(PrimitiveType type, string name, Transform parent)
        {
            var go = GameObject.CreatePrimitive(type);
            go.name = name;
            go.transform.SetParent(parent, false);
            var autoCollider = go.GetComponent<Collider>();
            if (autoCollider != null)
            {
                Object.DestroyImmediate(autoCollider);
            }
            return go;
        }
    }
}
