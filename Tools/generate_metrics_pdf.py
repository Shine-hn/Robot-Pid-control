"""Generate the metrics PDF (提出物②) from a telemetry CSV.

Usage:
    python generate_metrics_pdf.py <telemetry.csv> <output.pdf> [run_summary.json]

Reads the per-FixedUpdate telemetry CSV that TelemetryLogger.WriteCsv produces, plus the
run_summary.json that SimulationRunner writes alongside it, and builds the report the
assignment enumerates:

    走破時間 / 最大合成加速度 / 平均合成加速度 / 最大速度 / 最大角速度 / 最大ジャーク /
    加速度の時系列グラフ / Fixed Timestep

Two things the assignment leaves ambiguous are resolved by reporting BOTH rather than
picking one:
  * 合成加速度 could mean the camera-top point (the capped quantity) or the chassis, so
    every acceleration metric is given for both.
  * 走破時間 is the StartLine-touch -> GoalLine-clearance interval measured by RaceManager,
    NOT the telemetry file's total span (which also covers the pre-start runway and the
    post-goal tail). The summary JSON supplies the authoritative value; the telemetry span
    is reported separately as 計測記録全体 so the difference is explicit.

Labels are Japanese (assignment wording) with English in parentheses. Japanese glyphs come
from reportlab's built-in HeiseiKakuGo-W5 CID font and a system CJK font for matplotlib, so
no font files need shipping.
"""
import csv
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image,
)
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle

CAP = 1.0          # m/s^2 hard cap on camera-top horizontal resultant acceleration
JP_FONT = "HeiseiKakuGo-W5"


def setup_fonts():
    """Register a Japanese font for reportlab and pick a CJK-capable one for matplotlib."""
    pdfmetrics.registerFont(UnicodeCIDFont(JP_FONT))
    available = {f.name for f in font_manager.fontManager.ttflist}
    for candidate in ("Yu Gothic", "Meiryo", "MS Gothic", "BIZ UDGothic", "Noto Sans CJK JP"):
        if candidate in available:
            plt.rcParams["font.family"] = candidate
            return candidate
    return None  # graphs fall back to default font (axis labels stay ASCII-safe)


def load_csv(csv_path):
    cols = {}
    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for name in reader.fieldnames:
            cols[name] = []
        for row in reader:
            for name in reader.fieldnames:
                cols[name].append(float(row[name]))
    return cols


def load_summary(csv_path, explicit):
    """Prefer an explicit path, else run_summary.json beside the CSV. Missing => None."""
    path = explicit or os.path.join(os.path.dirname(os.path.abspath(csv_path)), "run_summary.json")
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    return None


def peak(cols, key):
    return max(cols[key]) if cols.get(key) else 0.0


def mean(cols, key):
    v = cols.get(key) or []
    return sum(v) / len(v) if v else 0.0


def make_figures(cols, out_dir, start_t, finish_t):
    t = cols["time_s"]

    # 1) Acceleration time series (both points) with the hard cap and the timed window.
    accel_png = os.path.join(out_dir, "_accel.png")
    plt.figure(figsize=(7.2, 3.1))
    plt.plot(t, cols["camera_top_accel_mps2"], color="#1f77b4", linewidth=1.2,
             label="camera-top |a|")
    plt.plot(t, cols["chassis_accel_mps2"], color="#7f7f7f", linewidth=0.9, alpha=0.75,
             label="chassis |a|")
    plt.axhline(CAP, color="#d62728", linestyle="--", linewidth=1.2, label="1.00 m/s² cap")
    if start_t is not None and finish_t is not None and finish_t > start_t:
        plt.axvspan(start_t, finish_t, color="#2ca02c", alpha=0.10)
        plt.axvline(start_t, color="#2ca02c", linewidth=1.0)
        plt.axvline(finish_t, color="#2ca02c", linewidth=1.0)
    plt.xlabel("time (s)")
    plt.ylabel("accel (m/s²)")
    plt.title("加速度の時系列 / Acceleration time series (shaded = timed run)")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    plt.savefig(accel_png, dpi=150)
    plt.close()

    # 2) Speed.
    speed_png = os.path.join(out_dir, "_speed.png")
    plt.figure(figsize=(7.2, 2.7))
    plt.plot(t, cols["chassis_speed_mps"], color="#2ca02c", linewidth=1.2, label="chassis speed")
    plt.plot(t, cols["camera_top_speed_mps"], color="#9467bd", linewidth=1.0, alpha=0.8,
             label="camera-top speed")
    plt.xlabel("time (s)")
    plt.ylabel("speed (m/s)")
    plt.title("速度の時系列 / Speed time series")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    plt.savefig(speed_png, dpi=150)
    plt.close()

    # 3) Jerk + angular speed (the other two scored quantities).
    jerk_png = os.path.join(out_dir, "_jerk.png")
    fig, ax1 = plt.subplots(figsize=(7.2, 2.7))
    ax1.plot(t, cols["camera_top_jerk_mps3"], color="#e377c2", linewidth=0.9)
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("jerk (m/s³)", color="#e377c2")
    ax1.grid(True, alpha=0.3)
    ax2 = ax1.twinx()
    ax2.plot(t, cols["angular_speed_degps"], color="#ff7f0e", linewidth=0.9)
    ax2.set_ylabel("angular speed (deg/s)", color="#ff7f0e")
    plt.title("ジャーク・角速度 / Jerk and angular speed")
    plt.tight_layout()
    plt.savefig(jerk_png, dpi=150)
    plt.close()

    # 4) Top-down path.
    path_png = os.path.join(out_dir, "_path.png")
    plt.figure(figsize=(4.6, 5.4))
    plt.plot(cols["pos_x_m"], cols["pos_z_m"], color="#ff7f0e", linewidth=1.6)
    plt.scatter([cols["pos_x_m"][0]], [cols["pos_z_m"][0]], c="green", s=30, zorder=5, label="spawn")
    plt.scatter([cols["pos_x_m"][-1]], [cols["pos_z_m"][-1]], c="red", s=30, zorder=5, label="end")
    plt.xlabel("X (m)")
    plt.ylabel("Z (m)")
    plt.title("走行経路 / Driven path")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="best", fontsize=8)
    plt.tight_layout()
    plt.savefig(path_png, dpi=150)
    plt.close()

    return accel_png, speed_png, jerk_png, path_png


def build_pdf(cols, summary, csv_path, out_pdf):
    setup_fonts()
    out_dir = os.path.dirname(os.path.abspath(out_pdf))
    os.makedirs(out_dir, exist_ok=True)

    telemetry_span = cols["time_s"][-1] if cols["time_s"] else 0.0
    if summary:
        course_time = summary.get("courseTimeSeconds", 0.0)
        fixed_dt = summary.get("fixedTimestepSeconds", 0.02)
        start_t = summary.get("startTimeSeconds")
        finish_t = summary.get("finishTimeSeconds")
        finished = summary.get("raceFinished", False)
        invalidated = summary.get("invalidated", False)
    else:
        course_time, fixed_dt, start_t, finish_t = float("nan"), 0.02, None, None
        finished, invalidated = False, False

    accel_png, speed_png, jerk_png, path_png = make_figures(cols, out_dir, start_t, finish_t)

    cam_peak, cam_mean = peak(cols, "camera_top_accel_mps2"), mean(cols, "camera_top_accel_mps2")
    ch_peak, ch_mean = peak(cols, "chassis_accel_mps2"), mean(cols, "chassis_accel_mps2")
    cap_status = "PASS" if cam_peak <= CAP else "FAIL"

    styles = getSampleStyleSheet()
    jp_title = ParagraphStyle("jpTitle", parent=styles["Title"], fontName=JP_FONT, fontSize=17)
    jp_body = ParagraphStyle("jpBody", parent=styles["Normal"], fontName=JP_FONT, fontSize=9)

    doc = SimpleDocTemplate(out_pdf, pagesize=A4,
                            leftMargin=16 * mm, rightMargin=16 * mm,
                            topMargin=14 * mm, bottomMargin=14 * mm)
    flow = [
        Paragraph("ロボット掃除機+見守りカメラ シミュレーション 計測結果", jp_title),
        Paragraph(
            "テレメトリ: {}（{} サンプル, {:.0f} Hz）&nbsp;&nbsp;結果: {}".format(
                os.path.basename(csv_path), len(cols["time_s"]), 1.0 / fixed_dt,
                "完走 (valid)" if finished and not invalidated else "未完走/無効",
            ), jp_body),
        Spacer(1, 5 * mm),
    ]

    data = [
        ["項目 (metric)", "カメラ頭頂部 (camera-top)", "車体 (chassis)"],
        ["走破時間 (course time)", "{:.3f} s".format(course_time), "—"],
        ["最大合成加速度 (max resultant accel)",
         "{:.3f} m/s²".format(cam_peak), "{:.3f} m/s²".format(ch_peak)],
        ["平均合成加速度 (avg resultant accel)",
         "{:.3f} m/s²".format(cam_mean), "{:.3f} m/s²".format(ch_mean)],
        ["最大速度 (max speed)",
         "{:.3f} m/s".format(peak(cols, "camera_top_speed_mps")),
         "{:.3f} m/s".format(peak(cols, "chassis_speed_mps"))],
        ["最大角速度 (max angular velocity)",
         "{:.1f} deg/s  ({:.3f} rad/s)".format(peak(cols, "angular_speed_degps"),
                                               peak(cols, "angular_speed_degps") * 3.14159265 / 180.0),
         "—"],
        ["最大ジャーク (max jerk)", "{:.2f} m/s³".format(peak(cols, "camera_top_jerk_mps3")), "—"],
        ["Fixed Timestep", "{:.4f} s  ({:.0f} Hz, 一定)".format(fixed_dt, 1.0 / fixed_dt), "—"],
        ["加速度上限 1.00 m/s² 適合 (cap compliance)", "{}  (peak {:.3f})".format(cap_status, cam_peak), "—"],
        ["計測記録全体 (telemetry span, 参考)", "{:.2f} s".format(telemetry_span), "—"],
    ]
    table = Table(data, colWidths=[74 * mm, 56 * mm, 48 * mm])
    cap_color = colors.HexColor("#1a7f37") if cap_status == "PASS" else colors.HexColor("#d1242f")
    table.setStyle(TableStyle([
        ("FONTNAME", (0, 0), (-1, -1), JP_FONT),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#2f3640")),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("FONTSIZE", (0, 0), (-1, -1), 8.5),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.grey),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f2f3f5")]),
        ("TEXTCOLOR", (1, 8), (1, 8), cap_color),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
    ]))
    flow.append(table)
    flow.append(Spacer(1, 3 * mm))
    flow.append(Paragraph(
        "※ 走破時間はスタートライン接触からゴールライン完全通過までの計測値。"
        "テレメトリ全体（スタート前の助走・ゴール後の惰走を含む）とは異なる。"
        "合成加速度は重力を除いた水平成分のみ。", jp_body))
    flow.append(Spacer(1, 4 * mm))

    usable_w = A4[0] - 32 * mm
    flow.append(Image(accel_png, width=usable_w, height=usable_w * 3.1 / 7.2))
    flow.append(Spacer(1, 2 * mm))
    flow.append(Image(speed_png, width=usable_w, height=usable_w * 2.7 / 7.2))
    flow.append(Spacer(1, 2 * mm))
    flow.append(Image(jerk_png, width=usable_w, height=usable_w * 2.7 / 7.2))
    flow.append(Spacer(1, 2 * mm))
    flow.append(Image(path_png, width=usable_w * 0.52, height=usable_w * 0.52 * 5.4 / 4.6))

    doc.build(flow)

    for p in (accel_png, speed_png, jerk_png, path_png):
        try:
            os.remove(p)
        except OSError:
            pass

    print("Wrote {} ({} bytes)".format(out_pdf, os.path.getsize(out_pdf)))
    print("  course time      = {:.3f} s".format(course_time))
    print("  fixed timestep   = {:.4f} s".format(fixed_dt))
    print("  cam-top accel    = max {:.3f} / avg {:.3f}  [{}]".format(cam_peak, cam_mean, cap_status))
    print("  chassis accel    = max {:.3f} / avg {:.3f}".format(ch_peak, ch_mean))


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    csv_path, out_pdf = sys.argv[1], sys.argv[2]
    explicit_summary = sys.argv[3] if len(sys.argv) > 3 else None

    cols = load_csv(csv_path)
    if not cols.get("time_s"):
        print("ERROR: no samples in", csv_path)
        sys.exit(1)

    summary = load_summary(csv_path, explicit_summary)
    if summary is None:
        print("WARNING: run_summary.json not found -- 走破時間 and Fixed Timestep "
              "cannot be reported authoritatively. Re-run the simulation to produce it.")
    build_pdf(cols, summary, csv_path, out_pdf)


if __name__ == "__main__":
    main()
