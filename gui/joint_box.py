"""
JointBox Widget

Self-contained Tkinter widget for displaying and controlling a single joint.
Shows both raw (from encoder) and logical (mapped) angles.
"""

import tkinter as tk
from tkinter import ttk

from angle_mapping import raw_to_logical, get_logical_limits, is_raw_in_range


class JointBox(ttk.LabelFrame):
    """Widget for a single joint: shows slider and current readings."""
    
    def __init__(self, parent, idx: int, cfg: dict):
        """
        Args:
            parent: Tkinter parent widget
            idx: Joint index (0-5)
            cfg: Joint config dict from config.JOINTS
        """
        self.base_title = cfg.get("label", f"Joint {idx+1}")
        super().__init__(parent, text=self.base_title)
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
        self.out_of_range = False

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

        # Out-of-range warning (hidden by default)
        self.oor_label = tk.Label(angle_frame, text="⚠ OUT OF RANGE",
                                  font=("TkDefaultFont", 8, "bold"),
                                  fg="white", bg="#D32F2F")
        # Not packed yet — shown/hidden via _set_out_of_range()

        # Slider
        self.scale = ttk.Scale(self, from_=self.min_angle, to=self.max_angle,
                       orient=tk.HORIZONTAL, variable=self.angle,
                       command=self._on_scale)
        self.scale.grid(row=1, column=0, sticky="ew", padx=6)

        # Min / Max labels at the ends of the slider
        label_row = ttk.Frame(self)
        label_row.grid(row=2, column=0, sticky="ew", padx=6)
        self.min_label = ttk.Label(label_row, text=f"{self.min_angle:.1f}°")
        self.min_label.pack(side="left")
        self.max_label = ttk.Label(label_row, text=f"{self.max_angle:.1f}°")
        self.max_label.pack(side="right")

        # Current value label
        self.val_label = ttk.Label(self, text=f"{start:.1f}°")
        self.val_label.grid(row=3, column=0, sticky="e", padx=6, pady=(2, 6))

        self.columnconfigure(0, weight=1)

    def _on_scale(self, _event=None):
        """Handle slider movement."""
        v = self.angle.get()
        self.val_label.config(text=f"{v:.1f}°")

    def update_current_angle(self, raw_angle: float):
        """Update the current angle display from telemetry data (raw from Teensy)."""
        self.current_raw_angle = raw_angle
        self.current_logical_angle = raw_to_logical(raw_angle, self.idx)
        self.logical_angle_label.config(text=f"Angle: {self.current_logical_angle:.1f}°")
        self.raw_angle_label.config(text=f"Raw: {raw_angle:.1f}°")
        self._set_out_of_range(not is_raw_in_range(raw_angle, self.idx))

    def _set_out_of_range(self, oor: bool):
        """Show or hide the out-of-range warning indicator."""
        if oor == self.out_of_range:
            return
        self.out_of_range = oor
        if oor:
            self.oor_label.pack(anchor="e")
        else:
            self.oor_label.pack_forget()
    
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

    def set_calibration_focus(self, is_active: bool, step_label=None):
        """Update the title so the active calibration joint stands out."""
        title = self.base_title
        if is_active:
            title += "  [CALIBRATING]"
            if step_label:
                title += f" {step_label.upper()}"
        self.config(text=title)

    def get_state(self) -> tuple:
        """Get current state.
        
        Returns:
            (enabled: bool, angle: float) - angle is logical degrees
        """
        return bool(self.enabled.get()), float(self.angle.get())
