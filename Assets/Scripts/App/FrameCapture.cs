using System.Collections;
using System.IO;
using UnityEngine;

namespace PIDReport.App
{
    // Deterministic offline frame capture for the video deliverable.
    //
    // Unity Recorder is an Editor-only package -- it cannot run inside a standalone player
    // build -- so the video is captured in-build instead: this component sets
    // Time.captureFramerate, which makes game time advance a fixed 1/fps per rendered frame
    // regardless of wall-clock speed, then saves one PNG per frame after rendering finishes.
    // Because time is slaved to the frame count, the run is fully deterministic and the PNG
    // sequence plays back at exactly real-time when encoded at the same fps -- no matter how
    // slow the per-frame PNG encode actually is. Encoding the sequence to mp4 is done
    // afterward by Tools/encode_video.py.
    //
    // Capturing requires a real rendering context, so this only does anything in a headful
    // run (Editor Play mode, or a standalone launched WITHOUT -batchmode/-nographics):
    // WaitForEndOfFrame never completes when nothing is being rendered.
    public class FrameCapture : MonoBehaviour
    {
        public string OutputDir;

        // 50 fps matches the 50 Hz (0.02 s) physics step one-to-one, so every rendered/
        // captured frame corresponds to exactly one FixedUpdate -- no aliasing between the
        // physics timeline and the video timeline.
        public int CaptureFramerate = 50;

        // Safety cap (~60 s at 50 fps) so a hang can never fill the disk with frames.
        public int MaxFrames = 3000;

        private bool capturing;
        private int frameIndex;

        public int FrameCount => frameIndex;

        public void Begin()
        {
            Directory.CreateDirectory(OutputDir);
            // Clear any stale frames from a previous run so the encoder never mixes two runs.
            foreach (var f in Directory.GetFiles(OutputDir, "frame_*.png")) File.Delete(f);

            Time.captureFramerate = CaptureFramerate;
            capturing = true;
            StartCoroutine(CaptureLoop());
        }

        public void End()
        {
            capturing = false;
            Time.captureFramerate = 0; // release time back to real-time
        }

        private IEnumerator CaptureLoop()
        {
            while (capturing && frameIndex < MaxFrames)
            {
                yield return new WaitForEndOfFrame();
                if (!capturing) break;

                Texture2D tex = ScreenCapture.CaptureScreenshotAsTexture();
                byte[] png = tex.EncodeToPNG();
                Destroy(tex);

                string path = Path.Combine(OutputDir, "frame_" + frameIndex.ToString("D5") + ".png");
                File.WriteAllBytes(path, png);
                frameIndex++;
            }
        }
    }
}
