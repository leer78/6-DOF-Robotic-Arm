#!/usr/bin/env python3
"""
6-DOF Robotic Arm GUI Controller

Main application - integrates modular components:
  - joint_box.py: JointBox widget for individual joint control
  - angle_mapping.py: Raw <-> Logical angle conversion
  - calibration.py: 3-step calibration state machine
  - serial_protocol.py: Two-lane packet communication
  - config.py: Centralized configuration
"""

import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime
from collections import deque
import logging
import queue

import config
import serial_protocol
from joint_box import JointBox
from angle_mapping import raw_to_logical, logical_to_raw
from calibration import CalibrationState

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Global serial connection
_serial_conn = None


class ArmGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("6-DOF Arm Controller")
        # Maximize the window (windowed fullscreen with borders and controls)
        self.state('zoomed')

        self.joint_boxes = []
        self.estop = tk.IntVar(value=0)
        # Current operating mode
        self.mode = getattr(config, 'DEFAULT_MODE', 0)
        label = config.MODE_LABELS.get(self.mode, "") if hasattr(config, 'MODE_LABELS') else ""
        self.mode_var = tk.StringVar(value=f"{self.mode} - {label}")

        # Packet logging (stores tuples of (timestamp, packet_string))
        self.sent_packets = deque(maxlen=50)  # Keep last 50 sent packets
        self.received_packets = deque(maxlen=50)  # Keep last 50 received packets

        # Thread-safe queue for telemetry data (listener thread -> main thread)
        self._telemetry_queue = queue.Queue(maxsize=100)

        # Serial connection reference
        self.serial_conn = None
        
        # Calibration state machine
        self._calib = CalibrationState()

        self._create_widgets()
        
        # Initialize serial connection and listener
        self._init_serial()
        
        # Start polling telemetry queue from main thread
        self._poll_telemetry_queue()

        # Clean shutdown on window close
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _init_serial(self):
        """Initialize serial connection and start listener thread."""
        global _serial_conn
        try:
            self.serial_conn = serial_protocol.connect_serial()
            _serial_conn = self.serial_conn
            serial_protocol.set_telemetry_handler(self._handle_telemetry)
            serial_protocol.start_listener(self.serial_conn)
            logger.info("Serial connection established and listener started")
        except serial_protocol.ProtocolError as e:
            logger.error(f"Failed to connect serial: {e}")
            messagebox.showwarning("Serial Connection", 
                f"Could not connect to serial port:\n{e}\n\nGUI will run in offline mode.")
            self.serial_conn = None
            #_serial_conn = None
    
    def _process_telemetry(self, line: str):
        """Process telemetry packet and update joint boxes with encoder values."""
        self._log_received_packet(line)
        
        try:
            parsed = serial_protocol.parse_packet(line)
            if parsed.get("TYPE") == "DATA" and parsed.get("CMD") == "JOINT_ANGLES":
                # Update each joint box with its encoder value
                for i in range(min(len(self.joint_boxes), 6)):
                    encoder_key = f"ENCODER_{i+1}_ANGLE"
                    if encoder_key in parsed:
                        try:
                            angle = float(parsed[encoder_key])
                            self.joint_boxes[i].update_current_angle(angle)
                        except (ValueError, TypeError):
                            pass
                
                # Calibration: button edge detection (1→0 captures)
                if self._calib.is_active:
                    btn = int(parsed.get("BUTTON", 0))
                    if self._calib.process_button(btn):
                        raw = float(parsed.get(f"ENCODER_{self._calib.joint + 1}_ANGLE", 0.0))
                        self._calib.capture(raw, self.joint_boxes)
                        
        except serial_protocol.ProtocolError:
            pass

    def _handle_telemetry(self, line: str):
        """Handle incoming DATA packets (telemetry) from Teensy.
        
        CRITICAL: This is called from the listener background thread!
        Do NOT call any tkinter methods here - just queue the data.
        """
        # Put data in thread-safe queue (non-blocking)
        try:
            self._telemetry_queue.put_nowait(line)
        except queue.Full:
            pass  # Drop oldest data if queue is full
    
    def _poll_telemetry_queue(self):
        """Poll the telemetry queue from main thread and process pending data."""
        # Process up to 10 items per poll to avoid blocking GUI
        for _ in range(10):
            try:
                line = self._telemetry_queue.get_nowait()
                self._process_telemetry(line)
            except queue.Empty:
                break
        # Schedule next poll (runs on main thread event loop)
        self.after(20, self._poll_telemetry_queue)

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

        # Bind variable traces so values can be used by command builders
        for box in self.joint_boxes:
            pass  # Connect to serial_protocol handlers later

        # Bottom controls
        bottom = ttk.Frame(main)
        bottom.pack(fill="x", pady=(10, 0))

        # Control bar - reorganized layout
        # Single frame for all controls in proper order
        control_frame = ttk.Frame(bottom)
        control_frame.pack(side="left", anchor="w", fill="x", expand=True, padx=4)

        # 1. ESTOP button (height=2 to match Send Command size)
        self.estop_btn = tk.Button(control_frame, text="ESTOP", command=self._do_estop,
                       bg="red", fg="white", font=("TkDefaultFont", 12, "bold"),
                       width=10, height=2)
        self.estop_btn.pack(side="left", padx=4, pady=2)

        # 2. MODE display (orange box, height=2 to match button size, fixed width)
        self.current_mode_label = tk.Label(control_frame, text=f"MODE: {config.MODE_LABELS.get(self.mode, 'UNKNOWN').upper()}", 
                                           bg="#ff9800", fg="white", font=("TkDefaultFont", 11, "bold"), 
                                           padx=12, pady=2, relief="raised", bd=2, height=2, width=20, anchor="center")
        self.current_mode_label.pack(side="left", padx=4, pady=2)

        # 3. Mode dropdown section
        mode_section = ttk.Frame(control_frame)
        mode_section.pack(side="left", padx=4)
        ttk.Label(mode_section, text="Mode:").pack(side="left", padx=(0,4))
        try:
            mode_options = [f"{k} - {v}" for k, v in sorted(config.MODE_LABELS.items())]
        except Exception:
            mode_options = ["0 - Idle", "1 - Calibration", "2 - Move", "3 - Reserved"]

        self.mode_menu = ttk.Combobox(mode_section, values=mode_options, textvariable=self.mode_var, state="readonly", width=20)
        try:
            self.mode_menu.current(int(self.mode))
        except Exception:
            pass
        self.mode_menu.pack(side="left")
        self.mode_menu.bind("<<ComboboxSelected>>", self._on_mode_changed)

        # 4. Command dropdown section
        cmd_section = ttk.Frame(control_frame)
        cmd_section.pack(side="left", padx=4)
        ttk.Label(cmd_section, text="Command:").pack(side="left", padx=(0,4))
        self.cmd_var = tk.StringVar(value="JOINTS_TO_ANGLE")
        self.cmd_menu = ttk.Combobox(cmd_section, textvariable=self.cmd_var, state="readonly", width=22)
        self.cmd_menu.pack(side="left")
        self._update_available_commands()

        # 5. Send Command button (height=2 to match ESTOP and MODE display, right-aligned)
        self.send_btn = tk.Button(control_frame, text="Send\nCommand", command=self._on_send,
                  bg="#2e7d32", fg="white", font=("TkDefaultFont", 11, "bold"),
                  width=20, height=2)
        self.send_btn.pack(side="right", padx=4, pady=2)

        # Serial info on second row
        serial_frame = ttk.Frame(bottom)
        serial_frame.pack(side="left", anchor="w", padx=4)
        port = config.SERIAL_PORT.get("PORT", "COM3")
        baud = config.SERIAL_PORT.get("BAUD", 115200)
        self.port_label = ttk.Label(serial_frame, text=f"Serial: {port}@{baud}")
        self.port_label.pack(side="left", padx=4)

        self.columnconfigure(0, weight=1)

        # ===== Packet Logging Section (bottom row) =====
        log_frame = ttk.Frame(main)
        log_frame.pack(fill="both", expand=True, pady=(10, 0))

        # Top: Sent Packets Log
        sent_label = ttk.Label(log_frame, text="Sent Packets (Most Recent First)", font=("TkDefaultFont", 10, "bold"))
        sent_label.grid(row=0, column=0, sticky="w", padx=0, pady=(0, 5))

        # Wrap sent text in a frame for internal padding
        sent_frame = tk.Frame(log_frame, bg="white", relief="sunken", bd=1)
        sent_frame.grid(row=1, column=0, sticky="nsew")

        self.sent_text = tk.Text(sent_frame, height=6, font=("Courier", 9), bg="#f5f5f5", wrap="none")
        self.sent_text.pack(side="left", fill="both", expand=True, padx=8, pady=8)

        sent_scroll = ttk.Scrollbar(sent_frame, orient="vertical", command=self.sent_text.yview)
        sent_scroll.pack(side="right", fill="y")
        self.sent_text.config(yscroll=sent_scroll.set)

        # Configure bold tag for timestamps
        self.sent_text.tag_configure("bold", font=("Courier", 9, "bold"))

        # Bottom: Received Packets Log
        recv_label = ttk.Label(log_frame, text="Received Packets (Most Recent First)", font=("TkDefaultFont", 10, "bold"))
        recv_label.grid(row=2, column=0, sticky="w", padx=0, pady=(10, 5))

        # Wrap received text in a frame for internal padding
        recv_frame = tk.Frame(log_frame, bg="white", relief="sunken", bd=1)
        recv_frame.grid(row=3, column=0, sticky="nsew")

        self.recv_text = tk.Text(recv_frame, height=6, font=("Courier", 9), bg="#f5f5f5", wrap="none")
        self.recv_text.pack(side="left", fill="both", expand=True, padx=8, pady=8)

        recv_scroll = ttk.Scrollbar(recv_frame, orient="vertical", command=self.recv_text.yview)
        recv_scroll.pack(side="right", fill="y")
        self.recv_text.config(yscroll=recv_scroll.set)

        # Configure bold tag for timestamps
        self.recv_text.tag_configure("bold", font=("Courier", 9, "bold"))

        # Configure grid weights for logging area
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(1, weight=1)
        log_frame.rowconfigure(3, weight=1)

    def _log_sent_packet(self, packet_str: str):
        """Log a sent packet with bold timestamp"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        packet_clean = packet_str.strip()
        
        self.sent_packets.appendleft(f"[{timestamp}] {packet_clean}\n")
        
        self.sent_text.config(state="normal")
        self.sent_text.insert("1.0", f"[{timestamp}] {packet_clean}\n")
        # Apply bold only to timestamp
        ts_len = len(timestamp) + 3  # [timestamp]
        self.sent_text.tag_add("bold", "1.0", f"1.{ts_len}")
        self.sent_text.see("1.0")

    def _log_received_packet(self, packet_str: str):
        """Log a received packet with bold timestamp"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        packet_clean = packet_str.strip()
        
        self.received_packets.appendleft(f"[{timestamp}] {packet_clean}\n")
        
        self.recv_text.config(state="normal")
        self.recv_text.insert("1.0", f"[{timestamp}] {packet_clean}\n")
        # Apply bold only to timestamp
        ts_len = len(timestamp) + 3  # [timestamp]
        self.recv_text.tag_add("bold", "1.0", f"1.{ts_len}")
        self.recv_text.see("1.0")

    def _update_available_commands(self):
        """Update command dropdown to show only commands available in current mode"""
        available_commands = []
        for cmd, schema in config.PROTOCOL_SCHEMAS["CMD"].items():
            allowed_modes = schema.get("allowed_modes", [])
            if self.mode in allowed_modes:
                available_commands.append(cmd)
        
        self.cmd_menu.config(values=available_commands)
        # If current selection is no longer available, pick the first one
        if self.cmd_var.get() not in available_commands:
            if available_commands:
                self.cmd_var.set(available_commands[0])
            else:
                self.cmd_var.set("")

    def _on_mode_changed(self, event=None):
        """Handle mode dropdown change - only update available commands, don't change actual mode"""
        try:
            sel = self.mode_menu.get()
            new_mode = int(sel.split()[0])
            # Just update available commands, don't change self.mode yet
            # Actual mode only changes when SET_MODE command is sent
            self._update_available_commands()
        except Exception:
            pass

    def _on_send(self):
        """Send command button handler - builds packet based on selected command"""
        try:
            selected_cmd = self.cmd_var.get()
            
            if selected_cmd == "SET_MODE":
                # Build SET_MODE command
                sel = self.mode_menu.get()
                try:
                    new_mode = int(sel.split()[0])
                except Exception:
                    messagebox.showerror("Mode error", f"Unable to parse selected mode: {sel}")
                    return
                
                packet = serial_protocol.build_packet(TYPE="CMD", CMD="SET_MODE", current_mode=self.mode, MODE=new_mode)
                
            elif selected_cmd == "JOINTS_TO_ANGLE":
                # Build JOINTS_TO_ANGLE command - convert logical slider values to raw
                kwargs = {"current_mode": self.mode}
                logical_angles = []
                for i, box in enumerate(self.joint_boxes):
                    _, logical_angle = box.get_state()
                    raw_angle = logical_to_raw(logical_angle, i)
                    kwargs[f"JOINT_{i+1}_ANG"] = round(raw_angle, 1)
                    logical_angles.append(f"J{i+1}={logical_angle:.1f}°")
                packet = serial_protocol.build_packet(TYPE="CMD", CMD="JOINTS_TO_ANGLE", **kwargs)
                # Log shows both raw (in packet) and logical (for readability)
                logger.info(f"Sending JOINTS_TO_ANGLE (logical): {', '.join(logical_angles)}")
                
            elif selected_cmd == "JOINT_EN":
                # Build JOINT_EN command
                kwargs = {"current_mode": self.mode}
                for i, box in enumerate(self.joint_boxes, 1):
                    enabled, _ = box.get_state()
                    kwargs[f"JOINT_{i}_EN"] = int(enabled)
                packet = serial_protocol.build_packet(TYPE="CMD", CMD="JOINT_EN", **kwargs)
                
            elif selected_cmd == "ESTOP":
                # Build ESTOP command
                packet = serial_protocol.build_packet(TYPE="CMD", CMD="ESTOP", current_mode=self.mode, STOP="ALL")
                
            elif selected_cmd == "CALIBRATE_JOINT":
                # Start calibration from joint 1
                self._calib.start()
                return  # No packet to send, just start listening
            else:
                messagebox.showerror("Error", f"Unknown command: {selected_cmd}")
                return
            
            # Send packet and wait for ACK
            self._log_sent_packet(packet)
            
            if self.serial_conn is None:
                messagebox.showwarning("Offline", f"Packet built:\n{packet.strip()}\n\n(No serial connection)")
                return
            
            try:
                serial_protocol.send_packet(self.serial_conn, packet)
                serial_protocol.wait_for_ack(packet, timeout=6.0)
                
                # ACK received - update UI state for SET_MODE
                if selected_cmd == "SET_MODE":
                    sel = self.mode_menu.get()
                    new_mode = int(sel.split()[0])
                    self.mode = new_mode
                    label_text = config.MODE_LABELS.get(self.mode, "UNKNOWN")
                    self.current_mode_label.config(text=f"MODE: {label_text.upper()}")
                    self._update_available_commands()
                
                self._log_received_packet(f"TYPE=ACK,CMD={selected_cmd} (verified)")
                logger.info(f"Command {selected_cmd} acknowledged")
                
            except serial_protocol.ProtocolError as e:
                messagebox.showerror("ACK Error", f"Failed to get ACK:\n{e}")
                logger.error(f"ACK error: {e}")
                
        except serial_protocol.ProtocolError as e:
            messagebox.showerror("Protocol Error", f"Failed to build packet:\n{str(e)}")

    def _do_estop(self):
        """ESTOP button handler - immediate stop"""
        try:
            self.estop.set(1)
            packet = serial_protocol.build_packet(TYPE="CMD", CMD="ESTOP", current_mode=self.mode, STOP="ALL")
            self._log_sent_packet(packet)
            
            if self.serial_conn is None:
                messagebox.showwarning("ESTOP (Offline)", f"ESTOP packet built:\n{packet.strip()}\n\n(No serial connection)")
                return
            
            try:
                serial_protocol.send_packet(self.serial_conn, packet)
                serial_protocol.wait_for_ack(packet, timeout=5.0)
                self._log_received_packet("TYPE=ACK,CMD=ESTOP (verified)")
                messagebox.showwarning("ESTOP", "ESTOP command acknowledged by Teensy")
                logger.info("ESTOP acknowledged")
            except serial_protocol.ProtocolError as e:
                messagebox.showerror("ESTOP Error", f"ESTOP sent but no ACK received:\n{e}")
                logger.error(f"ESTOP ACK error: {e}")
                
        except serial_protocol.ProtocolError as e:
            messagebox.showerror("Protocol Error", f"Failed to build ESTOP packet:\n{str(e)}")

    def _on_close(self):
        """Clean shutdown on window close"""
        try:
            serial_protocol.stop_listener()
            if self.serial_conn is not None:
                self.serial_conn.close()
                logger.info("Serial connection closed")
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
        self.destroy()


def main():
    app = ArmGUI()
    app.mainloop()


if __name__ == "__main__":
    main()