"""Encode a PNG frame sequence (captured in-build by FrameCapture) into an mp4.

Usage:
    python encode_video.py <frames_dir> <output_mp4> [fps]

Uses imageio's bundled ffmpeg (imageio-ffmpeg), so no system ffmpeg install is needed.
Frames must be named frame_00000.png, frame_00001.png, ... (zero-padded, sortable).
The fps must match FrameCapture.CaptureFramerate (default 50) for real-time playback.
"""
import sys
import os
import glob
import imageio.v2 as imageio


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)

    frames_dir = sys.argv[1]
    out_path = sys.argv[2]
    fps = int(sys.argv[3]) if len(sys.argv) > 3 else 50

    files = sorted(glob.glob(os.path.join(frames_dir, "frame_*.png")))
    if not files:
        print("ERROR: no frame_*.png files found in", frames_dir)
        sys.exit(1)

    out_dir = os.path.dirname(os.path.abspath(out_path))
    os.makedirs(out_dir, exist_ok=True)

    # libx264 + yuv420p for broad playback compatibility (QuickTime, browsers, PowerPoint).
    writer = imageio.get_writer(
        out_path,
        fps=fps,
        codec="libx264",
        quality=8,
        pixelformat="yuv420p",
        macro_block_size=None,
    )
    try:
        for f in files:
            writer.append_data(imageio.imread(f))
    finally:
        writer.close()

    size = os.path.getsize(out_path)
    print("Wrote {} ({} bytes) from {} frames at {} fps".format(out_path, size, len(files), fps))


if __name__ == "__main__":
    main()
