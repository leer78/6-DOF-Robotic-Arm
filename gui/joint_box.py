"""
JointBox Widget

Self-contained Tkinter widget for displaying and controlling a single joint.
Shows both raw (from encoder) and logical (mapped) angles.
"""

import tkinter as tk
from tkinter import ttk
import math

from angle_mapping import raw_to_logical, get_logical_limits


class JointBox(ttk.LabelFrame):
    """Widget for a single joint: shows angle dial, slider, and current readings."""
    
    def __init__(self, parent, idx: int, cfg: dict):
        """
        Args:
            parent: Tkinter parent widget
            idx: Joint index (0-5)
            cfg: Joint config dict from config.JOINTS
        """
        super().__init__(parent, text=cfg.get("label", f"Joint {idx+1}"))
        self.idx = idx
        # Compute logical limits from raw calibration values
        self.min_angle, self.max_angle = get_logical_limits(idx)
        start = cfg.get("ref_offset", 0.0)  # Start slider at reference position

        # Default enabled state comes from config (default 0 = disabled)
        self.enabled = tk.IntVar(value=cfg.get("enabled", 0))
        self.angle = tk.DoubleVar(value=start)
        
        # Current encoder angles (raw from Teensy, logical after mapping)
        self.current_raw_angle = 0.0
        self.current_logical_angle = 0.0

        self._build_ui(start)

    def _build_ui(self, start: float):
        """Build the widget UI."""
        # Top row: checkbox on left, angles on right
        top_row = ttk.Frame(self)
        top_row.grid(row=0, column=0, sticky="ew", padx=6, pady=(6, 2))
        top_row.columnconfigure(0, weight=1)
        
        self.chk = ttk.Checkbutton(top_row, text="Enable joint", variable=self.enabled)
        self.chk.pack(side="left")
        
        # Angle labels frame (stacked on right)
        angle_frame = ttk.Frame(top_row)
        angle_frame.pack(side="right")
        
        # Logical angle (primary, larger)
        self.logical_angle_label = ttk.Label(angle_frame, text="Angle: 0.0°", 
                                             font=("TkDefaultFont", 9, "bold"), 
                                             foreground="#2196F3")
        self.logical_angle_label.pack(anchor="e")
        
        # Raw angle (secondary, smaller, gray)
        self.raw_angle_label = ttk.Label(angle_frame, text="Raw: 0.0°", 
                                         font=("TkDefaultFont", 8), 
                                         foreground="#888888")
        self.raw_angle_label.pack(anchor="e")

        # Angle display canvas (circle + rotating line)
        self.canvas_size = 140
        self.canvas = tk.Canvas(self, width=self.canvas_size, height=self.canvas_size)
        self.canvas.grid(row=1, column=0, padx=6, pady=(2, 6))
        self._draw_canvas()

        # Slider
        self.scale = ttk.Scale(self, from_=self.min_angle, to=self.max_angle,
                       orient=tk.HORIZONTAL, variable=self.angle,
                       command=self._on_scale)
        self.scale.grid(row=2, column=0, sticky="ew", padx=6)

        # Min / Max labels at the ends of the slider
        label_row = ttk.Frame(self)
        label_row.grid(row=3, column=0, sticky="ew", padx=6)
        self.min_label = ttk.Label(label_row, text=f"{self.min_angle:.1f}°")
        self.min_label.pack(side="left")
        self.max_label = ttk.Label(label_row, text=f"{self.max_angle:.1f}°")
        self.max_label.pack(side="right")

        # Current value label
        self.val_label = ttk.Label(self, text=f"{start:.1f}°")
        self.val_label.grid(row=4, column=0, sticky="e", padx=6, pady=(2, 6))

        self.columnconfigure(0, weight=1)
        # draw initial angle line
        self.line_id = None
        self._update_canvas_line()

    def _on_scale(self, _event=None):
        """Handle slider movement."""
        v = self.angle.get()
        self.val_label.config(text=f"{v:.1f}°")
        try:
            self._update_canvas_line()
        except Exception:
            pass

    def _draw_canvas(self):
        """Draw base circle and quadrant labels."""
        self.canvas.delete("all")
        s = self.canvas_size
        cx = cy = s // 2
        r = int(s * 0.4)
        # circle
        self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, outline="black")
        # quadrant ticks and labels
        label_font = ("TkDefaultFont", 8)
        self.canvas.create_text(cx + r - 12, cy, text="0", anchor="center", font=label_font)
        self.canvas.create_text(cx, cy - r + 12, text="90", anchor="center", font=label_font)
        self.canvas.create_text(cx - r + 12, cy, text="180", anchor="center", font=label_font)
        self.canvas.create_text(cx, cy + r - 12, text="270", anchor="center", font=label_font)
        self.line_id = None

    def _update_canvas_line(self):
        """Update the rotating line on the angle dial."""
        ang = float(self.angle.get()) % 360.0
        s = self.canvas_size
        cx = cy = s // 2
        r = int(s * 0.4)
        rad = math.radians(ang)
        x = cx + r * math.cos(rad)
        y = cy - r * math.sin(rad)
        # remove previous line
        if self.line_id is not None:
            try:
                self.canvas.delete(self.line_id)
            except Exception:
                pass
        self.line_id = self.canvas.create_line(cx, cy, x, y, fill="red", width=2)
    
    def update_current_angle(self, raw_angle: float):
        """Update the current angle display from telemetry data (raw from Teensy)."""
        self.current_raw_angle = raw_angle
        self.current_logical_angle = raw_to_logical(raw_angle, self.idx)
        self.logical_angle_label.config(text=f"Angle: {self.current_logical_angle:.1f}°")
        self.raw_angle_label.config(text=f"Raw: {raw_angle:.1f}°")
    
    def update_range(self, min_deg: float = None, max_deg: float = None, start_deg: float = None):
        """Update slider range and labels after calibration (all values are logical degrees)."""
        if min_deg is not None:
            self.min_angle = min_deg
            self.scale.config(from_=min_deg)
            self.min_label.config(text=f"{min_deg:.1f}°")
        if max_deg is not None:
            self.max_angle = max_deg
            self.scale.config(to=max_deg)
            self.max_label.config(text=f"{max_deg:.1f}°")
        if start_deg is not None:
            self.angle.set(start_deg)
            self.val_label.config(text=f"{start_deg:.1f}°")
            self._update_canvas_line()

    def get_state(self) -> tuple:
        """Get current state.
        
        Returns:
            (enabled: bool, angle: float) - angle is logical degrees
        """
        return bool(self.enabled.get()), float(self.angle.get())
