#!/usr/bin/env python3
"""
Minimal Tkinter GUI to control 6 joints and send a combined serial packet.

Controls:
- 6 joint boxes (enable checkbox + slider per joint)
- Send Command button: sends a single CSV packet over serial
- ESTOP button: sets estop bit to 1 and immediately sends an ESTOP packet

Packet format (CSV):
ESTOP, j0_enable, j0_pos, j1_enable, j1_pos, ..., j5_enable, j5_pos\n
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import math

try:
    import serial
except Exception:
    serial = None

import config


class JointBox(ttk.LabelFrame):
    def __init__(self, parent, idx, cfg):
        super().__init__(parent, text=cfg.get("label", f"Joint {idx+1}"))
        self.idx = idx
        self.min_angle = cfg.get("min", 0.0)
        self.max_angle = cfg.get("max", 180.0)
        start = cfg.get("start", 0.0)

        # Default enabled state comes from config (default 0 = disabled)
        self.enabled = tk.IntVar(value=cfg.get("enabled", 0))
        self.angle = tk.DoubleVar(value=start)

        self.chk = ttk.Checkbutton(self, text="Enable joint", variable=self.enabled)
        self.chk.grid(row=0, column=0, sticky="w", padx=6, pady=(6, 2))

        # Angle display canvas (circle + rotating line)
        self.canvas_size = 140
        self.canvas = tk.Canvas(self, width=self.canvas_size, height=self.canvas_size)
        self.canvas.grid(row=1, column=0, padx=6, pady=(2, 6))
        # draw static parts of the canvas (circle + quadrant labels)
        self._draw_canvas()

        # Slider
        self.scale = ttk.Scale(self, from_=self.min_angle, to=self.max_angle,
                       orient=tk.HORIZONTAL, variable=self.angle,
                       command=self._on_scale)
        self.scale.grid(row=2, column=0, sticky="ew", padx=6)

        # Min / Max labels at the ends of the slider
        label_row = ttk.Frame(self)
        label_row.grid(row=3, column=0, sticky="ew", padx=6)
        min_label = ttk.Label(label_row, text=f"{self.min_angle:.1f}°")
        min_label.pack(side="left")
        max_label = ttk.Label(label_row, text=f"{self.max_angle:.1f}°")
        max_label.pack(side="right")

        # Current value label
        self.val_label = ttk.Label(self, text=f"{start:.1f}°")
        self.val_label.grid(row=4, column=0, sticky="e", padx=6, pady=(2, 6))

        self.columnconfigure(0, weight=1)
        # draw initial angle line
        self._update_canvas_line()

    def _on_scale(self, _event=None):
        v = self.angle.get()
        self.val_label.config(text=f"{v:.1f}°")
        # Update the canvas line to reflect angle (map to 0-360)
        try:
            self._update_canvas_line()
        except Exception:
            pass

    def _draw_canvas(self):
        # draw base circle and quadrant labels
        self.canvas.delete("all")
        s = self.canvas_size
        cx = cy = s // 2
        r = int(s * 0.4)
        # circle
        self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, outline="black")
        # quadrant ticks and labels: place slightly inside the circle and center them
        label_font = ("TkDefaultFont", 8)
        self.canvas.create_text(cx + r - 12, cy, text="0", anchor="center", font=label_font)
        self.canvas.create_text(cx, cy - r + 12, text="90", anchor="center", font=label_font)
        self.canvas.create_text(cx - r + 12, cy, text="180", anchor="center", font=label_font)
        self.canvas.create_text(cx, cy + r - 12, text="270", anchor="center", font=label_font)
        # initial line id placeholder
        self.line_id = None

    def _update_canvas_line(self):
        # angle from slider, map into [0,360)
        ang = float(self.angle.get()) % 360.0
        s = self.canvas_size
        cx = cy = s // 2
        r = int(s * 0.4)
        # convert degrees (0 at +X, CCW positive) to canvas coords (y down)
        rad = math.radians(ang)
        x = cx + r * math.cos(rad)
        y = cy - r * math.sin(rad)
        # remove previous line
        if getattr(self, "line_id", None) is not None:
            try:
                self.canvas.delete(self.line_id)
            except Exception:
                pass
        # draw new red line
        self.line_id = self.canvas.create_line(cx, cy, x, y, fill="red", width=2)


    def get_state(self):
        return bool(self.enabled.get()), float(self.angle.get())


class ArmGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("6-DOF Arm Controller")
        self.geometry("1280x1020")

        self.joint_boxes = []
        self.estop = tk.IntVar(value=0)
        # store last sent packet for preview
        self._last_packet = ""

        self._create_widgets()
        self._serial = None
        self._init_serial()

    def _create_widgets(self):
        main = ttk.Frame(self)
        main.pack(fill="both", expand=True, padx=8, pady=8)

        # Create a grid of 3 columns x 2 rows for joint boxes
        grid = ttk.Frame(main)
        grid.pack(fill="both", expand=True)

        num = config.NUM_JOINTS
        for i in range(num):
            cfg = config.JOINTS[i] if i < len(config.JOINTS) else {}
            box = JointBox(grid, i, cfg)
            r = i // 3
            c = i % 3
            box.grid(row=r, column=c, padx=6, pady=6, sticky="nsew")
            grid.columnconfigure(c, weight=1)
            grid.rowconfigure(r, weight=1)
            self.joint_boxes.append(box)

        # Bind variable traces so packet preview updates when sliders/checks change
        for box in self.joint_boxes:
            try:
                box.angle.trace_add('write', lambda *a: self._update_packet_preview())
                box.enabled.trace_add('write', lambda *a: self._update_packet_preview())
            except Exception:
                # fallback for older tkinter
                box.angle.trace('w', lambda *a: self._update_packet_preview())
                box.enabled.trace('w', lambda *a: self._update_packet_preview())

        # update preview when estop toggles
        try:
            self.estop.trace_add('write', lambda *a: self._update_packet_preview())
        except Exception:
            self.estop.trace('w', lambda *a: self._update_packet_preview())

        # Bottom controls
        bottom = ttk.Frame(main)
        bottom.pack(fill="x", pady=(10, 0))

        # Left side: ESTOP and serial port label
        left_frame = ttk.Frame(bottom)
        left_frame.pack(side="left", anchor="w")

        # ESTOP - use a tk.Button so we can style background color reliably
        self.estop_btn = tk.Button(left_frame, text="ESTOP", command=self._do_estop,
                       bg="red", fg="white", font=("TkDefaultFont", 12, "bold"),
                       width=10, height=2)
        self.estop_btn.pack(side="left", padx=4)

        # Serial settings label
        port = config.SERIAL_PORT.get("PORT", "COM3")
        baud = config.SERIAL_PORT.get("BAUD", 115200)
        self.port_label = ttk.Label(left_frame, text=f"Serial: {port}@{baud}")
        self.port_label.pack(side="left", padx=10)

        # Right side: send button and packet preview
        send_frame = ttk.Frame(bottom)
        send_frame.pack(side="right", anchor="e")

        # Send Command - larger green button
        self.send_btn = tk.Button(send_frame, text="Send Command", command=self._on_send,
                      bg="#2e7d32", fg="white", font=("TkDefaultFont", 11, "bold"),
                      width=14, height=2)
        self.send_btn.pack(side="left", padx=(4, 10))

        # Packet preview area
        packet_frame = ttk.Frame(send_frame)
        packet_frame.pack(side="left", padx=4)

        self.current_packet_label = ttk.Label(packet_frame, text="", font=("Courier", 9))
        self.current_packet_label.pack(side="top", anchor="e")

        self.previous_packet_label = ttk.Label(packet_frame, text="previous packet: ", font=("TkDefaultFont", 8), foreground="gray")
        self.previous_packet_label.pack(side="top", anchor="e")

        # initialize packet preview
        self._update_packet_preview()

    def _init_serial(self):
        if config.SERIAL_PORT.get("DRY_RUN", False):
            print("GUI in DRY_RUN mode: packets will be printed to console, not sent.")
            return

        if serial is None:
            messagebox.showwarning("pyserial missing",
                                   "pyserial is not installed. GUI will print packets to console.")
            return

        port = config.SERIAL_PORT.get("PORT", "COM3")
        baud = config.SERIAL_PORT.get("BAUD", 115200)
        try:
            self._serial = serial.Serial(port, baud, timeout=0.1)
            # Give the serial port a moment to initialize
            time.sleep(0.1)
        except Exception as e:
            messagebox.showwarning("Serial open failed", f"Unable to open serial port {port}: {e}\nPackets will be printed.")
            self._serial = None

    def _format_packet(self):
        # Build CSV packet: ESTOP, j0_enable, j0_pos, j1_enable, j1_pos, ...\n
        fields = [str(int(self.estop.get()))]
        for box in self.joint_boxes:
            enabled, angle = box.get_state()
            fields.append(str(int(enabled)))
            # Format angle to one decimal place
            fields.append(f"{angle:.1f}")

        packet = ",".join(fields) + "\n"
        return packet

    def _update_packet_preview(self):
        packet = self._format_packet().strip()
        # show current packet (what will be sent)
        if hasattr(self, 'current_packet_label'):
            self.current_packet_label.config(text=packet)
        # show previous packet (smaller)
        if hasattr(self, 'previous_packet_label'):
            prev = self._last_packet.strip() if self._last_packet else ""
            display = f"previous packet: {prev}" if prev else "previous packet:"
            self.previous_packet_label.config(text=display)

    def _send_packet(self, packet: str):
        if config.SERIAL_PORT.get("DRY_RUN", False) or self._serial is None:
            print("DRY_RUN / no serial: ", packet.strip())
            return True

        try:
            self._serial.write(packet.encode("utf-8"))
            return True
        except Exception as e:
            messagebox.showerror("Serial write failed", f"Failed to send packet: {e}")
            return False

    def _on_send(self):
        packet = self._format_packet()
        ok = self._send_packet(packet)
        if ok:
            # store as last packet and update preview
            self._last_packet = packet
            self._update_packet_preview()
            messagebox.showinfo("Sent", "Command packet sent.")

    def _do_estop(self):
        # Set estop bit = 1 and immediately send packet
        self.estop.set(1)
        packet = self._format_packet()
        self._send_packet(packet)
        messagebox.showwarning("ESTOP", "ESTOP sent. Motors should be disabled.")


def main():
    app = ArmGUI()
    app.mainloop()


if __name__ == "__main__":
    main()