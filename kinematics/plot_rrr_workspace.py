#!/usr/bin/env python3
"""
Monte Carlo workspace plotter for the arm's planar RRR study.

What it shows
-------------
- Blue dots: wrist center / wrist-point workspace (position used by the IK
  position solver; independent of Joint 5 when J4 is fixed).
- Orange dots: tool-tip workspace with J4 fixed at 0 and Joint 5 randomized.

The script samples random logical joint angles inside the CURRENT calibrated
GUI limits for J2, J3, and J5, computes forward kinematics, and opens a simple
Tkinter window showing the reachable XZ-plane points.

Usage
-----
    python kinematics\\plot_rrr_workspace.py
    python kinematics\\plot_rrr_workspace.py --samples 100
    python kinematics\\plot_rrr_workspace.py --samples 100 --seed 42
    python kinematics\\plot_rrr_workspace.py --samples 100 --no-window
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys
import tkinter as tk


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GUI_DIR = os.path.join(REPO_ROOT, "gui")
KIN_DIR = os.path.join(REPO_ROOT, "kinematics")
if GUI_DIR not in sys.path:
    sys.path.insert(0, GUI_DIR)
if KIN_DIR not in sys.path:
    sys.path.insert(0, KIN_DIR)

from angle_mapping import get_logical_limits  # noqa: E402
from ik_symbolic import DH_PARAMS  # noqa: E402


D1 = DH_PARAMS["d1"]
A2 = DH_PARAMS["a2"]
A3 = DH_PARAMS["a3"]
D4 = DH_PARAMS["d4"]
D6 = DH_PARAMS["d6"]


def dh_transform(theta: float, d: float, a: float, alpha: float) -> list[list[float]]:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ]


def mat4_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
        for i in range(4)
    ]


def point_from_t(t: list[list[float]]) -> tuple[float, float, float]:
    return (t[0][3], t[1][3], t[2][3])


def fk_rrr_points(j2_deg: float, j3_deg: float, j5_deg: float) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return (wrist_center_xyz, tool_tip_xyz) for logical joint angles.

    Model assumptions:
    - J1 fixed at 0
    - J4 fixed at 0
    - J6 fixed at 0
    - Angles are GUI logical angles, converted using the same convention as gui/ik_solver.py
    """
    q1 = 0.0
    q2 = math.radians(j2_deg) - math.pi / 2.0
    q3 = math.radians(j3_deg)
    q4 = math.radians(0.0) + math.pi
    q5 = math.radians(j5_deg)
    q6 = 0.0

    t01 = dh_transform(q1, D1, 0.0, -math.pi / 2.0)
    t12 = dh_transform(q2, 0.0, A2, 0.0)
    t23 = dh_transform(q3, 0.0, A3, -math.pi / 2.0)
    t34 = dh_transform(q4, D4, 0.0, -math.pi / 2.0)
    t45 = dh_transform(q5, 0.0, 0.0, math.pi / 2.0)
    t56 = dh_transform(q6, D6, 0.0, 0.0)

    t03 = mat4_mul(mat4_mul(t01, t12), t23)
    t04 = mat4_mul(t03, t34)
    t06 = mat4_mul(mat4_mul(t04, t45), t56)

    wrist_center = point_from_t(t04)
    tool_tip = point_from_t(t06)
    return wrist_center, tool_tip


def sample_workspace(samples: int, seed: int | None = None) -> tuple[list[tuple[float, float]], list[tuple[float, float]], dict[str, tuple[float, float]]]:
    rng = random.Random(seed)
    limits = {
        "J2": get_logical_limits(1),
        "J3": get_logical_limits(2),
        "J5": get_logical_limits(4),
    }

    wrist_points: list[tuple[float, float]] = []
    tool_points: list[tuple[float, float]] = []

    for _ in range(samples):
        j2 = rng.uniform(*limits["J2"])
        j3 = rng.uniform(*limits["J3"])
        j5 = rng.uniform(*limits["J5"])
        wrist_xyz, tool_xyz = fk_rrr_points(j2, j3, j5)
        wrist_points.append((wrist_xyz[0], wrist_xyz[2]))
        tool_points.append((tool_xyz[0], tool_xyz[2]))

    return wrist_points, tool_points, limits


def canvas_xy(x: float, z: float, bounds: tuple[float, float, float, float], width: int, height: int, pad: int) -> tuple[float, float]:
    min_x, max_x, min_z, max_z = bounds
    span_x = max(max_x - min_x, 1.0)
    span_z = max(max_z - min_z, 1.0)
    px = pad + (x - min_x) / span_x * (width - 2 * pad)
    py = height - pad - (z - min_z) / span_z * (height - 2 * pad)
    return px, py


def world_x_from_canvas(px: float, bounds: tuple[float, float, float, float], width: int, pad: int) -> float:
    min_x, max_x, _, _ = bounds
    span_x = max(max_x - min_x, 1.0)
    return min_x + (px - pad) / max(width - 2 * pad, 1.0) * span_x


def world_z_from_canvas(py: float, bounds: tuple[float, float, float, float], height: int, pad: int) -> float:
    _, _, min_z, max_z = bounds
    span_z = max(max_z - min_z, 1.0)
    return min_z + (height - pad - py) / max(height - 2 * pad, 1.0) * span_z


def nice_tick_step(span: float, target_ticks: int = 8) -> float:
    raw = max(span / max(target_ticks, 1), 1.0)
    magnitude = 10 ** math.floor(math.log10(raw))
    normalized = raw / magnitude
    if normalized <= 1.0:
        nice = 1.0
    elif normalized <= 2.0:
        nice = 2.0
    elif normalized <= 5.0:
        nice = 5.0
    else:
        nice = 10.0
    return nice * magnitude


def tick_values(min_v: float, max_v: float, step: float) -> list[float]:
    start = math.floor(min_v / step) * step
    end = math.ceil(max_v / step) * step
    vals = []
    cur = start
    while cur <= end + 1e-9:
        vals.append(cur)
        cur += step
    return vals


def draw_workspace_window(wrist_points: list[tuple[float, float]], tool_points: list[tuple[float, float]], limits: dict[str, tuple[float, float]], samples: int) -> None:
    all_points = wrist_points + tool_points
    min_x = min(x for x, _ in all_points)
    max_x = max(x for x, _ in all_points)
    min_z = min(z for _, z in all_points)
    max_z = max(z for _, z in all_points)

    # Add a bit of plot padding in mm.
    margin = 20.0
    bounds = (min_x - margin, max_x + margin, min_z - margin, max_z + margin)

    width, height, pad = 1000, 760, 70
    root = tk.Tk()
    root.title("RRR Workspace Monte Carlo (XZ plane)")
    canvas = tk.Canvas(root, width=width, height=height, bg="white")
    canvas.pack(fill="both", expand=True)
    status_var = tk.StringVar(
        value="Hover near a point to see its exact X/Z coordinates."
    )
    status_label = tk.Label(root, textvariable=status_var, anchor="w", font=("Courier", 10))
    status_label.pack(fill="x", padx=10, pady=(0, 8))

    # Axes and tick marks
    x_axis_y = canvas_xy(0.0, 0.0, bounds, width, height, pad)[1]
    z_axis_x = canvas_xy(0.0, 0.0, bounds, width, height, pad)[0]
    canvas.create_rectangle(pad, pad, width - pad, height - pad, outline="#cfcfcf")
    canvas.create_line(pad, x_axis_y, width - pad, x_axis_y, fill="#9e9e9e", width=1)
    canvas.create_line(z_axis_x, height - pad, z_axis_x, pad, fill="#9e9e9e", width=1)

    x_step = nice_tick_step(bounds[1] - bounds[0])
    z_step = nice_tick_step(bounds[3] - bounds[2])
    for x_tick in tick_values(bounds[0], bounds[1], x_step):
        px, _ = canvas_xy(x_tick, bounds[2], bounds, width, height, pad)
        canvas.create_line(px, height - pad, px, pad, fill="#f0f0f0")
        canvas.create_line(px, height - pad, px, height - pad + 6, fill="#616161")
        canvas.create_text(px, height - pad + 18, text=f"{x_tick:.0f}", font=("Courier", 9))
    for z_tick in tick_values(bounds[2], bounds[3], z_step):
        _, py = canvas_xy(bounds[0], z_tick, bounds, width, height, pad)
        canvas.create_line(pad, py, width - pad, py, fill="#f0f0f0")
        canvas.create_line(pad - 6, py, pad, py, fill="#616161")
        canvas.create_text(pad - 28, py, text=f"{z_tick:.0f}", font=("Courier", 9))

    # Labels
    canvas.create_text(width / 2, 24, text=f"RRR Workspace Monte Carlo — {samples} samples", font=("TkDefaultFont", 14, "bold"))
    canvas.create_text(width / 2, 48, text="Blue = wrist center / Orange = tool tip (J4 fixed at 0)", font=("TkDefaultFont", 10))
    canvas.create_text(width / 2, height - 20, text="X (mm)", font=("TkDefaultFont", 11, "bold"))
    canvas.create_text(20, height / 2, text="Z (mm)", angle=90, font=("TkDefaultFont", 11, "bold"))

    # Draw points
    plotted_points: list[tuple[float, float, float, float, str]] = []
    for x, z in wrist_points:
        px, py = canvas_xy(x, z, bounds, width, height, pad)
        canvas.create_oval(px - 3, py - 3, px + 3, py + 3, fill="#1565c0", outline="")
        plotted_points.append((px, py, x, z, "wrist"))
    for x, z in tool_points:
        px, py = canvas_xy(x, z, bounds, width, height, pad)
        canvas.create_oval(px - 3, py - 3, px + 3, py + 3, fill="#ef6c00", outline="")
        plotted_points.append((px, py, x, z, "tool"))

    hover_marker = canvas.create_oval(-10, -10, -5, -5, outline="#212121", width=2)

    # Legend / info box
    j2_lo, j2_hi = limits["J2"]
    j3_lo, j3_hi = limits["J3"]
    j5_lo, j5_hi = limits["J5"]
    info = (
        f"Joint limits (logical deg)\n"
        f"J2: [{j2_lo:.1f}, {j2_hi:.1f}]\n"
        f"J3: [{j3_lo:.1f}, {j3_hi:.1f}]\n"
        f"J5: [{j5_lo:.1f}, {j5_hi:.1f}]\n\n"
        f"Wrist X range: {min(x for x, _ in wrist_points):.1f} to {max(x for x, _ in wrist_points):.1f} mm\n"
        f"Wrist Z range: {min(z for _, z in wrist_points):.1f} to {max(z for _, z in wrist_points):.1f} mm"
    )
    canvas.create_rectangle(width - 315, 70, width - 30, 210, outline="#9e9e9e", fill="#fafafa")
    canvas.create_text(width - 300, 80, text=info, anchor="nw", font=("Courier", 9))

    def on_motion(event: tk.Event) -> None:
        cursor_x = world_x_from_canvas(event.x, bounds, width, pad)
        cursor_z = world_z_from_canvas(event.y, bounds, height, pad)

        best = None
        best_d2 = float("inf")
        for px, py, x, z, kind in plotted_points:
            d2 = (event.x - px) ** 2 + (event.y - py) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best = (px, py, x, z, kind)

        if best is not None and best_d2 <= 12 ** 2:
            px, py, x, z, kind = best
            canvas.coords(hover_marker, px - 6, py - 6, px + 6, py + 6)
            status_var.set(
                f"Nearest point: {kind:<5}  X={x:7.2f} mm  Z={z:7.2f} mm   "
                f"(cursor X={cursor_x:7.2f}, Z={cursor_z:7.2f})"
            )
        else:
            canvas.coords(hover_marker, -10, -10, -5, -5)
            status_var.set(
                f"Cursor: X={cursor_x:7.2f} mm  Z={cursor_z:7.2f} mm   "
                f"(move closer to a dot for exact plotted coordinates)"
            )

    canvas.bind("<Motion>", on_motion)

    root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot the arm's RRR workspace in the XZ plane.")
    parser.add_argument("--samples", type=int, default=100, help="Monte Carlo samples to generate (default: 100)")
    parser.add_argument("--seed", type=int, default=None, help="Optional RNG seed for repeatable samples")
    parser.add_argument("--no-window", action="store_true", help="Print sampled ranges without opening the Tk window")
    args = parser.parse_args()

    wrist_points, tool_points, limits = sample_workspace(args.samples, args.seed)

    if args.no_window:
        print(f"Samples: {args.samples}")
        for name in ("J2", "J3", "J5"):
            lo, hi = limits[name]
            print(f"{name} logical range: [{lo:.1f}, {hi:.1f}] deg")
        print(f"Wrist X range: {min(x for x, _ in wrist_points):.1f} to {max(x for x, _ in wrist_points):.1f} mm")
        print(f"Wrist Z range: {min(z for _, z in wrist_points):.1f} to {max(z for _, z in wrist_points):.1f} mm")
        print(f"Tool  X range: {min(x for x, _ in tool_points):.1f} to {max(x for x, _ in tool_points):.1f} mm")
        print(f"Tool  Z range: {min(z for _, z in tool_points):.1f} to {max(z for _, z in tool_points):.1f} mm")
        return

    draw_workspace_window(wrist_points, tool_points, limits, args.samples)


if __name__ == "__main__":
    main()
