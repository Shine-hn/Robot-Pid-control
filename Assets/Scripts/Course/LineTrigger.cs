using System;
using System.Collections.Generic;
using UnityEngine;
using PIDReport.Robot;

namespace PIDReport.Course
{
    // Fires when the robot's collider enters/exits this trigger volume. Checks for a
    // RobotRig in the OTHER collider's parent hierarchy rather than a tag on `other`
    // directly -- the robot's own collider lives on a child ("Body") of the tagged
    // root, and tags do not propagate to children, so a direct tag check on `other`
    // would silently never fire (the same class of bug as brief bug #2).
    //
    // Exit alone is NOT sufficient to call a line "crossed": OnTriggerExit fires just as
    // readily when the body backs out the way it came in. The assignment defines the finish
    // as "ゴールのラインを超えて完全に離れた時点" -- passing BEYOND the line and fully
    // separating from it -- so CrossedThrough additionally requires that the body left on
    // the opposite side of the line's plane from the one it arrived on.
    [RequireComponent(typeof(Collider))]
    public class LineTrigger : MonoBehaviour
    {
        public event Action<Collider> Entered;
        public event Action<Collider> Exited;

        // Full clearance in the direction of travel: entered on one side, left on the other.
        public event Action<Collider> CrossedThrough;

        private readonly Dictionary<Collider, float> entrySide = new Dictionary<Collider, float>();

        void OnTriggerEnter(Collider other)
        {
            if (other.GetComponentInParent<RobotRig>() == null) return;
            entrySide[other] = SignedSide(other);
            Entered?.Invoke(other);
        }

        void OnTriggerExit(Collider other)
        {
            if (other.GetComponentInParent<RobotRig>() == null) return;
            Exited?.Invoke(other);

            if (!entrySide.TryGetValue(other, out float enteredSide)) return;
            entrySide.Remove(other);

            float exitedSide = SignedSide(other);
            // Opposite signs => genuinely passed through the plane. Same sign => backed out.
            if (enteredSide * exitedSide < 0f) CrossedThrough?.Invoke(other);
        }

        // Signed position along the line's crossing axis, measured from the line's centre.
        // At the moment of enter/exit the body is roughly its own radius clear of the thin
        // slab, so this is comfortably non-zero on both sides -- no sign ambiguity.
        private float SignedSide(Collider other)
        {
            return Vector3.Dot(other.bounds.center - transform.position, CrossingAxis());
        }

        // The axis a body must travel along to cross the line is the slab's THINNEST local
        // axis. Detecting it from the box dimensions (rather than hardcoding local X) keeps
        // this correct for a line laid out along any axis.
        private Vector3 CrossingAxis()
        {
            var box = GetComponent<Collider>() as BoxCollider;
            if (box == null) return transform.right;

            Vector3 s = box.size;
            if (s.x <= s.y && s.x <= s.z) return transform.right;
            if (s.z <= s.x && s.z <= s.y) return transform.forward;
            return transform.up;
        }
    }
}
