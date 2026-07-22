using System.Collections.Generic;
using UnityEngine;

namespace PIDReport.Trajectory
{
    // A time-parameterized reference path: a sequence of segments, each starting
    // exactly where the previous one ended, evaluated as a single continuous timeline.
    public class RobotTrajectory
    {
        private readonly List<TrajectorySegment> segments;
        private readonly float[] segmentStartTimes;

        public float TotalDuration { get; }
        public IReadOnlyList<TrajectorySegment> Segments => segments;

        public RobotTrajectory(List<TrajectorySegment> segments)
        {
            this.segments = segments;
            segmentStartTimes = new float[segments.Count];
            float t = 0f;
            for (int i = 0; i < segments.Count; i++)
            {
                segmentStartTimes[i] = t;
                t += segments[i].Duration;
            }
            TotalDuration = t;
        }

        public TrajectoryState Evaluate(float t)
        {
            t = Mathf.Clamp(t, 0f, TotalDuration);
            int index = segments.Count - 1;
            for (int i = 0; i < segments.Count; i++)
            {
                if (t <= segmentStartTimes[i] + segments[i].Duration)
                {
                    index = i;
                    break;
                }
            }
            float localT = t - segmentStartTimes[index];
            return segments[index].Evaluate(localT);
        }
    }
}
