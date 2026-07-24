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

            // Single physics collider for the whole robot. A convex MeshCollider hull of
            // Unity's default low-poly (~20-side) Cylinder primitive was tried first but
            // produces persistent low-amplitude contact jitter at rest: the flat bottom
            // face is really a ring of a few dozen large planar facets, and PhysX's
            // resting contact manifold shifts between adjacent facet vertices frame to
            // frame, injecting small but real torque impulses every step. Those are far
            // too small to move the chassis noticeably (chassis linear accel stayed clean
            // throughout), but the camera-top point sits on a 0.5 m lever arm above the
            // CoM, and finite-differencing GetPointVelocity() there amplifies that same
            // jitter into spurious multi-m/s^2 spikes -- confirmed via a diagnostic
            // full-course run where camera-top acceleration spiked to 6.2 m/s^2 while
            // chassis acceleration stayed under 0.6 the entire time.
            // A BoxCollider was tried next and eliminated the jitter, but a box
            // circumscribing a 0.15 m-radius circle has corners out at 0.15*sqrt(2) =
            // 0.212 m -- 41% further out than the true footprint at diagonal headings --
            // and that extra reach clipped WallWest during a turn that the true circular
            // footprint clears, invalidating a run that should have finished.
            // The actual fix: a purpose-built, higher-resolution convex cylinder mesh
            // (32 sides vs. the default's ~20) used ONLY for the collider, not the visual
            // mesh. Smaller facets shrink the jitter amplitude, and every vertex still
            // sits at exactly the true 0.15 m radius (worst-case edge-midpoint shortfall
            // is cos(pi/32) ~= 0.48%, not the box's 41% overshoot) -- accurate footprint
            // and stable contact at the same time.
            var bodyCollider = body.AddComponent<MeshCollider>();
            bodyCollider.sharedMesh = GenerateCylinderColliderMesh(32);
            bodyCollider.convex = true;

            // The drive controller's chassis force already IS the abstracted net
            // wheel-traction force ("sufficient friction for traction is assumed" --
            // brief). Unity's default PhysicsMaterial has nonzero friction, which would
            // additionally treat the chassis as a block sliding against the floor and
            // resist that force on top of it -- effectively double-counting resistance
            // the brief says to ignore ("ignore rolling resistance"), and strong enough
            // (with this robot's weight) to fully cancel a safely torque-limited drive
            // force. Zero it out so the floor/walls only constrain motion, never resist it.
            bodyCollider.material = new PhysicsMaterial("Frictionless")
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

            // Default solver iteration counts (6 position / 1 velocity) leave real
            // per-step residual error in the resting contact solve for this body -- a
            // squat, high-CoM (0.5 m), zero-friction puck with a 32-sided convex hull
            // floor contact. That residual shows up as small frame-to-frame contact
            // jitter which, amplified through the camera-top point's 0.5 m lever arm,
            // still produced spurious acceleration spikes even after raising the hull's
            // facet count. More solver iterations is a pure numerical-accuracy increase
            // (better convergence on the same physical model), not a change to the
            // dynamics itself -- unlike damping, it doesn't resist real motion.
            rb.solverIterations = 16;
            rb.solverVelocityIterations = 8;

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

        // Builds a convex cylinder mesh (radius 0.5, height 2, y in [-1,1], centered at
        // the local origin) matching Unity's built-in Cylinder primitive's own unit
        // convention, so it composes correctly with the same localScale already applied
        // to the visual body -- but with `sides` facets instead of the primitive's fixed
        // low-poly count. Since MeshCollider.convex=true has PhysX cook its own convex
        // hull from the vertex positions alone, exact triangle winding doesn't affect
        // collision correctness -- only the vertex positions (and thus the hull shape) do.
        private static Mesh GenerateCylinderColliderMesh(int sides)
        {
            const float radius = 0.5f;
            const float halfHeight = 1f;

            var vertices = new System.Collections.Generic.List<Vector3>();
            var triangles = new System.Collections.Generic.List<int>();
            var topRing = new int[sides];
            var bottomRing = new int[sides];

            for (int i = 0; i < sides; i++)
            {
                float angle = i * Mathf.PI * 2f / sides;
                float x = Mathf.Cos(angle) * radius;
                float z = Mathf.Sin(angle) * radius;
                topRing[i] = vertices.Count;
                vertices.Add(new Vector3(x, halfHeight, z));
                bottomRing[i] = vertices.Count;
                vertices.Add(new Vector3(x, -halfHeight, z));
            }

            int topCenter = vertices.Count;
            vertices.Add(new Vector3(0, halfHeight, 0));
            int bottomCenter = vertices.Count;
            vertices.Add(new Vector3(0, -halfHeight, 0));

            for (int i = 0; i < sides; i++)
            {
                int next = (i + 1) % sides;
                triangles.Add(topRing[i]); triangles.Add(topRing[next]); triangles.Add(bottomRing[i]);
                triangles.Add(topRing[next]); triangles.Add(bottomRing[next]); triangles.Add(bottomRing[i]);
                triangles.Add(topCenter); triangles.Add(topRing[i]); triangles.Add(topRing[next]);
                triangles.Add(bottomCenter); triangles.Add(bottomRing[next]); triangles.Add(bottomRing[i]);
            }

            var mesh = new Mesh();
            mesh.SetVertices(vertices);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
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
