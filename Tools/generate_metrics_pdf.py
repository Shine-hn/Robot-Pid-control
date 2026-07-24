"""Generate the metrics PDF (deliverable #2) from a telemetry CSV.

Usage:
    python generate_metrics_pdf.py <telemetry.csv> <output.pdf>

Reads the per-FixedUpdate telemetry CSV that TelemetryLogger.WriteCsv produces and builds
a report PDF containing:
  * a run-summary stats table (course time, peak/mean camera-top acceleration vs the
    1.00 m/s^2 hard cap, peak speeds, peak jerk, peak angular speed),
  * a camera-top horizontal-acceleration time series with the 1.00 m/s^2 cap drawn on it,
  * chassis vs camera-top speed over time,
  * a top-down (X-Z) plot of the driven path.

Pure stdlib CSV parsing (no pandas). Matplotlib renders the figures; reportlab assembles
the PDF.
"""
import csv
import sys
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image,
)
from reportlab.lib.styles import getSampleStyleSheet

CAP = 1.0  # m/s^2 hard cap on camera-top horizontal resultant acceleration


def load(csv_path):
    cols = {}
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for name in reader.fieldnames:
            cols[name] = []
        for row in reader:
            for name in reader.fieldnames:
                cols[name].append(float(row[name]))
    return cols


def _peak(cols, key):
    return max(cols[key]) if cols.get(key) else 0.0


def _mean(cols, key):
    v = cols.get(key) or []
    return sum(v) / len(v) if v else 0.0


def make_figures(cols, out_dir):
    t = cols["time_s"]

    # 1) Camera-top acceleration vs time, with the hard cap line.
    accel_png = os.path.join(out_dir, "_accel.png")
    plt.figure(figsize=(7.2, 3.0))
    plt.plot(t, cols["camera_top_accel_mps2"], color="#1f77b4", linewidth=1.2,
             label="camera-top |a| (m/s²)")
    plt.axhline(CAP, color="#d62728", linestyle="--", linewidth=1.2,
                label="1.00 m/s² cap")
    plt.xlabel("time (s)")
    plt.ylabel("accel (m/s²)")
    plt.title("Camera-top horizontal acceleration")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    plt.savefig(accel_png, dpi=150)
    plt.close()

    # 2) Chassis vs camera-top speed over time.
    speed_png = os.path.join(out_dir, "_speed.png")
    plt.figure(figsize=(7.2, 3.0))
    plt.plot(t, cols["chassis_speed_mps"], color="#2ca02c", linewidth=1.2,
             label="chassis speed (m/s)")
    plt.plot(t, cols["camera_top_speed_mps"], color="#9467bd", linewidth=1.2,
             label="camera-top speed (m/s)")
    plt.xlabel("time (s)")
    plt.ylabel("speed (m/s)")
    plt.title("Chassis and camera-top speed")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    plt.savefig(speed_png, dpi=150)
    plt.close()

    # 3) Top-down driven path.
    path_png = os.path.join(out_dir, "_path.png")
    plt.figure(figsize=(4.6, 5.6))
    plt.plot(cols["pos_x_m"], cols["pos_z_m"], color="#ff7f0e", linewidth=1.6)
    plt.scatter([cols["pos_x_m"][0]], [cols["pos_z_m"][0]], c="green", s=30, zorder=5, label="start")
    plt.scatter([cols["pos_x_m"][-1]], [cols["pos_z_m"][-1]], c="red", s=30, zorder=5, label="end")
    plt.xlabel("X (m)")
    plt.ylabel("Z (m)")
    plt.title("Driven path (top-down)")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="best", fontsize=8)
    plt.tight_layout()
    plt.savefig(path_png, dpi=150)
    plt.close()

    return accel_png, speed_png, path_png


def build_pdf(cols, csv_path, out_pdf):
    out_dir = os.path.dirname(os.path.abspath(out_pdf))
    os.makedirs(out_dir, exist_ok=True)
    accel_png, speed_png, path_png = make_figures(cols, out_dir)

    peak_accel = _peak(cols, "camera_top_accel_mps2")
    mean_accel = _mean(cols, "camera_top_accel_mps2")
    duration = cols["time_s"][-1] if cols["time_s"] else 0.0
    cap_status = "PASS" if peak_accel <= CAP else "FAIL"

    styles = getSampleStyleSheet()
    doc = SimpleDocTemplate(out_pdf, pagesize=A4,
                            leftMargin=18 * mm, rightMargin=18 * mm,
                            topMargin=16 * mm, bottomMargin=16 * mm)
    flow = []
    flow.append(Paragraph("Robot Control Simulation — Run Metrics", styles["Title"]))
    flow.append(Paragraph("Source telemetry: " + os.path.basename(csv_path)
                          + " &nbsp;(" + str(len(cols["time_s"])) + " samples @ 50 Hz)",
                          styles["Normal"]))
    flow.append(Spacer(1, 6 * mm))

    data = [
        ["Metric", "Value", "Note"],
        ["Run duration (telemetry)", "{:.2f} s".format(duration), ""],
        ["Peak camera-top acceleration", "{:.3f} m/s²".format(peak_accel),
         "cap 1.00 m/s² — " + cap_status],
        ["Mean camera-top acceleration", "{:.3f} m/s²".format(mean_accel), ""],
        ["Peak camera-top jerk", "{:.2f} m/s³".format(_peak(cols, "camera_top_jerk_mps3")), ""],
        ["Peak chassis speed", "{:.3f} m/s".format(_peak(cols, "chassis_speed_mps")), ""],
        ["Peak camera-top speed", "{:.3f} m/s".format(_peak(cols, "camera_top_speed_mps")), ""],
        ["Peak angular speed", "{:.1f} deg/s".format(_peak(cols, "angular_speed_degps")), ""],
    ]
    table = Table(data, colWidths=[70 * mm, 45 * mm, 55 * mm])
    cap_color = colors.HexColor("#1a7f37") if cap_status == "PASS" else colors.HexColor("#d1242f")
    table.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#2f3640")),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
        ("FONTSIZE", (0, 0), (-1, -1), 9),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.grey),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f2f3f5")]),
        ("TEXTCOLOR", (2, 2), (2, 2), cap_color),
        ("FONTNAME", (2, 2), (2, 2), "Helvetica-Bold"),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
    ]))
    flow.append(table)
    flow.append(Spacer(1, 6 * mm))

    usable_w = A4[0] - 36 * mm
    flow.append(Image(accel_png, width=usable_w, height=usable_w * 3.0 / 7.2))
    flow.append(Spacer(1, 3 * mm))
    flow.append(Image(speed_png, width=usable_w, height=usable_w * 3.0 / 7.2))
    flow.append(Spacer(1, 3 * mm))
    flow.append(Image(path_png, width=usable_w * 0.62, height=usable_w * 0.62 * 5.6 / 4.6))

    doc.build(flow)

    for p in (accel_png, speed_png, path_png):
        try:
            os.remove(p)
        except OSError:
            pass

    print("Wrote {} ({} bytes). Peak cam-top accel {:.3f} m/s^2 [{}].".format(
        out_pdf, os.path.getsize(out_pdf), peak_accel, cap_status))


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    csv_path, out_pdf = sys.argv[1], sys.argv[2]
    cols = load(csv_path)
    if not cols.get("time_s"):
        print("ERROR: no samples in", csv_path)
        sys.exit(1)
    build_pdf(cols, csv_path, out_pdf)


if __name__ == "__main__":
    main()
