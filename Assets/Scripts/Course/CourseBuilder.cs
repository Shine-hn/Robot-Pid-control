using UnityEngine;

namespace PIDReport.Course
{
    // Builds the confirmed course geometry at runtime (never a hand-authored scene,
    // for the same reason the robot isn't: no risk of stray Editor-only state).
    // All coordinates are already-converted Unity units from the spec's mm grid.
    public static class CourseBuilder
    {
        public const float WallHeight = 0.30f;
        public const float LineTriggerHeight = 1.20f; // tall enough to catch the robot at any height, including the pole
        public const float LineTriggerThickness = 0.02f;

        public static readonly Vector3 RobotSpawnPosition = new Vector3(2.10f, 0f, 0.30f);
        public static readonly Quaternion RobotSpawnRotation = Quaternion.LookRotation(Vector3.left, Vector3.up); // heading -X

        private struct WallSpec
        {
            public readonly string Name;
            public readonly float CenterX, CenterZ, SizeX, SizeZ;
            public WallSpec(string name, float cx, float cz, float sx, float sz)
            {
                Name = name; CenterX = cx; CenterZ = cz; SizeX = sx; SizeZ = sz;
            }
        }

        private static readonly WallSpec[] Walls =
        {
            new WallSpec("WallSouth", 1.20f, -0.025f, 2.50f, 0.05f),
            new WallSpec("WallNorth", 1.20f, 3.025f, 2.50f, 0.05f),
            new WallSpec("WallWest", -0.025f, 1.50f, 0.05f, 3.10f),
            new WallSpec("WallEast", 2.425f, 1.50f, 0.05f, 3.10f),
            new WallSpec("LowerBlock", 1.50f, 0.90f, 1.80f, 0.60f),
            new WallSpec("UpperBlock", 0.90f, 2.10f, 1.80f, 0.60f),
        };

        public static GameObject BuildCourse()
        {
            var root = new GameObject("Course");

            BuildFloor(root.transform);

            foreach (var w in Walls)
            {
                BuildWall(w, root.transform);
            }

            // StartLine: spec (600,2400)-(600,3000) -> X=0.60, Z in [0, 0.6]
            BuildLineTrigger("StartLine", new Vector3(0.60f, LineTriggerHeight * 0.5f, 0.30f),
                new Vector3(LineTriggerThickness, LineTriggerHeight, 0.60f), root.transform);

            // GoalLine: spec (1800,0)-(1800,600) -> X=1.80, Z in [2.4, 3.0]
            BuildLineTrigger("GoalLine", new Vector3(1.80f, LineTriggerHeight * 0.5f, 2.70f),
                new Vector3(LineTriggerThickness, LineTriggerHeight, 0.60f), root.transform);

            return root;
        }

        private static void BuildFloor(Transform parent)
        {
            var floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.name = "Floor";
            floor.transform.SetParent(parent, false);
            floor.transform.localPosition = new Vector3(1.20f, -0.05f, 1.50f);
            floor.transform.localScale = new Vector3(2.60f, 0.10f, 3.20f); // top face at Y=0

            // The wheel/floor friction pair must be declared on BOTH surfaces. Leaving the
            // floor on Unity's default material (mu = 0.6) and combining with Average
            // silently produced mu = (0.12 + 0.6)/2 = 0.36 -- about 35 N of stiction against
            // a 15 N drive clamp, which pinned the robot in place. Both surfaces now carry
            // the same declared coefficients, so the combine mode cannot distort them.
            floor.GetComponent<Collider>().material = new PhysicsMaterial("WheelFloorTraction")
            {
                dynamicFriction = Robot.RobotRig.WheelFloorFriction,
                staticFriction = Robot.RobotRig.WheelFloorStaticFriction,
                bounciness = 0f,
                frictionCombine = PhysicsMaterialCombine.Average,
                bounceCombine = PhysicsMaterialCombine.Minimum
            };
        }

        private static void BuildWall(WallSpec w, Transform parent)
        {
            var wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
            wall.name = w.Name;
            wall.tag = "Wall";
            wall.transform.SetParent(parent, false);
            wall.transform.localPosition = new Vector3(w.CenterX, WallHeight * 0.5f, w.CenterZ);
            wall.transform.localScale = new Vector3(w.SizeX, WallHeight, w.SizeZ);
        }

        private static void BuildLineTrigger(string tagAndName, Vector3 localPosition, Vector3 size, Transform parent)
        {
            var go = new GameObject(tagAndName);
            go.tag = tagAndName;
            go.transform.SetParent(parent, false);
            go.transform.localPosition = localPosition;
            var box = go.AddComponent<BoxCollider>();
            box.size = size;
            box.isTrigger = true;
            go.AddComponent<LineTrigger>();
        }
    }
}
