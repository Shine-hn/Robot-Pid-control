using System;
using UnityEngine;
using PIDReport.Robot;

namespace PIDReport.Course
{
    // Fires when the robot's collider enters/exits this trigger volume. Checks for a
    // RobotRig in the OTHER collider's parent hierarchy rather than a tag on `other`
    // directly -- the robot's own collider lives on a child ("Body") of the tagged
    // root, and tags do not propagate to children, so a direct tag check on `other`
    // would silently never fire (the same class of bug as brief bug #2).
    [RequireComponent(typeof(Collider))]
    public class LineTrigger : MonoBehaviour
    {
        public event Action<Collider> Entered;
        public event Action<Collider> Exited;

        void OnTriggerEnter(Collider other)
        {
            if (other.GetComponentInParent<RobotRig>() != null) Entered?.Invoke(other);
        }

        void OnTriggerExit(Collider other)
        {
            if (other.GetComponentInParent<RobotRig>() != null) Exited?.Invoke(other);
        }
    }
}
