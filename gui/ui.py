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

try:
    import serial
except Exception:
    serial = None

from gui import config


class JointBox(ttk.LabelFrame):
    def __init__(self, parent, idx, cfg):
        super().__init__(parent, text=cfg.get("label", f"Joint {idx+1}"))
        self.idx = idx
        self.min_angle = cfg.get("min", -180.0)
        self.max_angle = cfg.get("max", 180.0)
        start = cfg.get("start", 0.0)

        self.enabled = tk.IntVar(value=1)
        self.angle = tk.DoubleVar(value=start)

        self.chk = ttk.Checkbutton(self, text="Enable joint", variable=self.enabled)
        self.chk.grid(row=0, column=0, sticky="w", padx=6, pady=(6, 2))

        # Slider
        self.scale = ttk.Scale(self, from_=self.min_angle, to=self.max_angle,
                               orient=tk.HORIZONTAL, variable=self.angle,
                               command=self._on_scale)
        self.scale.grid(row=1, column=0, sticky="ew", padx=6)

        # Current value label
        self.val_label = ttk.Label(self, text=f"{start:.1f}°")
        self.val_label.grid(row=2, column=0, sticky="e", padx=6, pady=(2, 6))

        self.columnconfigure(0, weight=1)

    def _on_scale(self, _event=None):
        v = self.angle.get()
        self.val_label.config(text=f"{v:.1f}°")

    def get_state(self):
        return bool(self.enabled.get()), float(self.angle.get())


class ArmGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("6-DOF Arm Controller")
        self.geometry("900x600")

        self.joint_boxes = []
        self.estop = tk.IntVar(value=0)

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

        # Bottom controls
        bottom = ttk.Frame(main)
        bottom.pack(fill="x", pady=(10, 0))

        # ESTOP
        self.estop_btn = ttk.Button(bottom, text="ESTOP", command=self._do_estop)
        self.estop_btn.pack(side="left", padx=4)

        # Send Command
        self.send_btn = ttk.Button(bottom, text="Send Command", command=self._on_send)
        self.send_btn.pack(side="right", padx=4)

        # Serial settings label
        port = config.SERIAL_PORT.get("PORT", "COM3")
        baud = config.SERIAL_PORT.get("BAUD", 115200)
        self.port_label = ttk.Label(bottom, text=f"Serial: {port}@{baud}")
        self.port_label.pack(side="left", padx=10)

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
