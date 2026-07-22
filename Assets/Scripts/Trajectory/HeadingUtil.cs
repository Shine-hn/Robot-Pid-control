using UnityEngine;

namespace PIDReport.Trajectory
{
    // Shared heading/rotation math matching Unity's own left-handed Y-rotation
    // convention (positive angle turns forward from +Z toward +X, i.e. clockwise
    // viewed from above) -- verified against transform.forward/atan2 behavior.
    public static class HeadingUtil
    {
        public static Vector3 ToForward(float headingRadians) =>
            new Vector3(Mathf.Sin(headingRadians), 0f, Mathf.Cos(headingRadians));

        public static Vector3 ToRight(float headingRadians) =>
            new Vector3(Mathf.Cos(headingRadians), 0f, -Mathf.Sin(headingRadians));

        public static float FromForward(Vector3 forward) => Mathf.Atan2(forward.x, forward.z);

        public static Vector3 RotateY(Vector3 v, float radians)
        {
            float s = Mathf.Sin(radians);
            float c = Mathf.Cos(radians);
            return new Vector3(v.x * c + v.z * s, v.y, -v.x * s + v.z * c);
        }
    }
}
