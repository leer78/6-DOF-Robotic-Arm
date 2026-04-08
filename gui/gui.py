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
import debug_logger
from joint_box import JointBox
from angle_mapping import raw_to_logical, logical_to_raw, is_raw_in_range
from calibration import CalibrationState
import ik_solver

# ── PRESET_1: Vertical pole sweep (x=140mm, ry=-179°) ───────────────────────
# Two IK-verified poses that trace a vertical line 115mm apart.
# Both have >15° margin from every joint limit (J2/J3/J4/J5).
_PRESET1_POINTS = [
    (140.0, 0.0, 155.0, 0.0, -179.0, 0.0),  # BOTTOM  J2=+1.2 J3=+60.3 J5=+29.5
    (140.0, 0.0, 270.0, 0.0, -179.0, 0.0),  # TOP     J2=-7.4 J3=+27.8 J5=+70.6
]
_PRESET1_NAMES    = ["BOTTOM", "TOP"]
_PRESET1_DELAY_MS = 5_000  # ms between moves

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Initialize file-based debug log as early as possible
debug_logger.init()

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

        # When True, sync sliders to live encoder readings on the next telemetry packet.
        # Set when entering MOVE mode so the first JOINTS_TO_ANGLE reflects reality
        # instead of jumping from the GUI's initialized slider position.
        self._sync_sliders_on_next_telem = False

        # Telemetry logging throttle: only log every Nth packet to reduce log spam
        self._telem_log_counter = 0
        self._telem_log_interval = 5  # Log every 5th telemetry packet (~100ms at 50Hz)

        # Packet logging (stores tuples of (timestamp, packet_string))
        self.sent_packets = deque(maxlen=50)  # Keep last 50 sent packets
        self.received_packets = deque(maxlen=50)  # Keep last 50 received packets

        # Thread-safe queue for telemetry data (listener thread -> main thread)
        self._telemetry_queue = queue.Queue(maxsize=100)

        # Serial connection reference
        self.serial_conn = None
        
        # Calibration state machine
        self._calib = CalibrationState()
        self._calib_session_values = {}
        self._last_calib_raw = None
        self._last_calib_logical = None
        self._last_calib_button_state = None

        # Preset state
        self._preset1_running = False
        self._preset1_step = 0        # 0 = BOTTOM, 1 = TOP
        self._preset1_after_id = None

        self._create_widgets()
        self._refresh_calibration_ui()
        
        # Initialize serial connection and listener
        self._init_serial()
        
        # Start polling telemetry queue from main thread
        self._poll_telemetry_queue()

        # Clean shutdown on window close
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _init_serial(self):
        """Initialize serial connection and start listener thread."""
        global _serial_conn
        port = config.SERIAL_PORT.get("PORT", "?")
        baud = config.SERIAL_PORT.get("BAUD", 115200)
        try:
            self.serial_conn = serial_protocol.connect_serial()
            _serial_conn = self.serial_conn
            serial_protocol.set_telemetry_handler(self._handle_telemetry)
            serial_protocol.start_listener(self.serial_conn)
            logger.info("Serial connection established and listener started")
            debug_logger.log_serial_connect(port, baud, success=True)
        except serial_protocol.ProtocolError as e:
            logger.error(f"Failed to connect serial: {e}")
            debug_logger.log_serial_connect(port, baud, success=False, error=str(e))
            messagebox.showwarning("Serial Connection", 
                f"Could not connect to serial port:\n{e}\n\nGUI will run in offline mode.")
            self.serial_conn = None
    
    def _process_telemetry(self, line: str):
        """Process telemetry packet and update joint boxes with encoder values."""
        self._log_received_packet(line)
        
        try:
            parsed = serial_protocol.parse_packet(line)
            if parsed.get("TYPE") == "DATA" and parsed.get("CMD") == "JOINT_ANGLES":
                # Update each joint box with its encoder value
                raw_angles = []
                logical_angles = []
                enabled_states = []
                for i in range(min(len(self.joint_boxes), 6)):
                    encoder_key = f"ENCODER_{i+1}_ANGLE"
                    if encoder_key in parsed:
                        try:
                            angle = float(parsed[encoder_key])
                            self.joint_boxes[i].update_current_angle(angle)
                            raw_angles.append(angle)
                            logical_angles.append(self.joint_boxes[i].current_logical_angle)
                        except (ValueError, TypeError):
                            raw_angles.append(float("nan"))
                            logical_angles.append(float("nan"))
                    else:
                        raw_angles.append(float("nan"))
                        logical_angles.append(float("nan"))
                    en, _ = self.joint_boxes[i].get_state()
                    enabled_states.append(bool(en))

                # Log joint positions in MOVE (2) and CALIBRATION (1) modes
                # Throttle to every Nth packet to reduce log spam
                if self.mode in (1, 2):
                    self._telem_log_counter += 1
                    if self._telem_log_counter >= self._telem_log_interval:
                        self._telem_log_counter = 0
                        button = None
                        try:
                            button = int(parsed.get("BUTTON", 0))
                        except (ValueError, TypeError):
                            pass
                        debug_logger.log_joint_positions(
                            mode=self.mode,
                            raw_angles=raw_angles,
                            logical_angles=logical_angles,
                            enabled=enabled_states,
                            button=button,
                        )

                # On first telemetry after entering MOVE mode, snap sliders to actual
                # encoder positions so the first JOINTS_TO_ANGLE is a no-op (no jump).
                if self._sync_sliders_on_next_telem:
                    self._sync_sliders_on_next_telem = False
                    for box in self.joint_boxes:
                        logical = box.current_logical_angle
                        box.angle.set(logical)
                        box.val_label.config(text=f"{logical:.1f}°")
                    logger.info("Sliders synced to encoder positions on MOVE entry")
                    debug_logger.log_event("Sliders synced to encoder positions on MOVE entry")
                
                # Calibration: button edge detection (1→0 captures)
                if self._calib.is_active:
                    btn = int(parsed.get("BUTTON", 0))
                    active_joint = self._calib.joint
                    active_step = self._calib.step
                    raw = None
                    logical = None
                    if active_joint is not None:
                        try:
                            raw = float(parsed.get(f"ENCODER_{active_joint + 1}_ANGLE", 0.0))
                        except (ValueError, TypeError):
                            raw = None
                        if active_joint < len(self.joint_boxes):
                            logical = self.joint_boxes[active_joint].current_logical_angle

                    if self._calib.process_button(btn):
                        capture_raw = 0.0 if raw is None else raw
                        self._record_calibration_capture(active_joint, active_step, capture_raw)
                        calibration_complete = self._calib.capture(capture_raw, self.joint_boxes)
                        if calibration_complete:
                            self._refresh_calibration_ui(
                                current_raw=capture_raw,
                                current_logical=logical,
                                button_state=btn,
                                status_override="Calibration complete - all enabled joints captured"
                            )
                        else:
                            captured_label = CalibrationState.STEP_LABELS[active_step]
                            self._refresh_calibration_ui(
                                current_raw=capture_raw,
                                current_logical=logical,
                                button_state=btn,
                                status_override=f"Captured {captured_label} for Joint {active_joint + 1}"
                            )
                    else:
                        self._refresh_calibration_ui(
                            current_raw=raw,
                            current_logical=logical,
                            button_state=btn
                        )
                elif self.mode == 1:
                    self._refresh_calibration_ui()

            elif parsed.get("TYPE") == "DATA" and parsed.get("CMD") == "PID_DEBUG":
                debug_logger.log_pid_debug(parsed)
                        
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

        # 5. Grip Control section (slider for SG90 end effector)
        grip_section = ttk.Frame(control_frame)
        grip_section.pack(side="left", padx=8)
        ttk.Label(grip_section, text="Grip Control:", font=("TkDefaultFont", 10, "bold")).pack(side="left", padx=(0, 4))
        self.grip_var = tk.IntVar(value=90)  # Default to middle position
        self.grip_slider = ttk.Scale(grip_section, from_=68, to=112, orient="horizontal", 
                                      length=150, variable=self.grip_var, command=self._on_grip_slider_changed)
        self.grip_slider.pack(side="left", padx=2)
        self.grip_value_label = ttk.Label(grip_section, text="90°", width=5)
        self.grip_value_label.pack(side="left", padx=2)

        # 6. Send Command button (height=2 to match ESTOP and MODE display, right-aligned)
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

        # Row for Calibration + IK panels side by side
        assist_row = ttk.Frame(main)
        assist_row.pack(fill="x", pady=(10, 0))

        calib_frame = ttk.LabelFrame(assist_row, text="Calibration Assistant")
        calib_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))
        calib_frame.columnconfigure(0, weight=1)

        self.calib_status_label = tk.Label(
            calib_frame,
            text="Calibration inactive",
            bg="#616161",
            fg="white",
            font=("TkDefaultFont", 10, "bold"),
            padx=10,
            pady=6,
            anchor="w"
        )
        self.calib_status_label.grid(row=0, column=0, sticky="ew", padx=8, pady=(8, 4))

        self.calib_instruction_var = tk.StringVar(
            value="Switch to CALIBRATION mode to begin measuring encoder values."
        )
        self.calib_instruction_label = ttk.Label(
            calib_frame,
            textvariable=self.calib_instruction_var,
            wraplength=1200,
            justify="left"
        )
        self.calib_instruction_label.grid(row=1, column=0, sticky="ew", padx=8)

        self.calib_live_var = tk.StringVar(value="Live reading: waiting for calibration")
        self.calib_live_label = ttk.Label(
            calib_frame,
            textvariable=self.calib_live_var,
            font=("Courier", 9)
        )
        self.calib_live_label.grid(row=2, column=0, sticky="w", padx=8, pady=(6, 0))

        self.calib_button_var = tk.StringVar(value="Hardware button: waiting for telemetry")
        self.calib_button_label = ttk.Label(calib_frame, textvariable=self.calib_button_var)
        self.calib_button_label.grid(row=3, column=0, sticky="w", padx=8, pady=(2, 0))

        self.calib_captures_var = tk.StringVar(value="Captured values: none yet")
        self.calib_captures_label = ttk.Label(
            calib_frame,
            textvariable=self.calib_captures_var,
            font=("Courier", 9)
        )
        self.calib_captures_label.grid(row=4, column=0, sticky="w", padx=8, pady=(2, 8))

        # Right: Inverse Kinematics panel
        self._create_ik_panel(assist_row)

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
        
        debug_logger.log_sent(packet_clean)

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
        
        debug_logger.log_received(packet_clean)

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

    def _reset_calibration_session(self):
        """Clear calibration session data used by the assistant panel."""
        self._calib_session_values = {}
        self._last_calib_raw = None
        self._last_calib_logical = None
        self._last_calib_button_state = None

    def _record_calibration_capture(self, joint_idx: int, step_idx: int, raw_angle: float):
        """Remember captured values so the operator can see progress during this run."""
        if joint_idx is None or step_idx is None:
            return
        joint_values = self._calib_session_values.setdefault(joint_idx, {})
        joint_values[CalibrationState.STEP_NAMES[step_idx]] = raw_angle

    def _format_calibration_angle(self, value):
        """Format live calibration values while handling missing telemetry cleanly."""
        if value is None:
            return "--"
        if isinstance(value, float) and value != value:
            return "--"
        return f"{value:.2f}°"

    def _get_calibration_instruction(self) -> str:
        """Human-readable prompt for the current calibration step."""
        if self._calib.is_active:
            joint_idx = self._calib.joint
            joint_name = self.joint_boxes[joint_idx].base_title
            step_prompts = {
                CalibrationState.STEP_REF: "Move to the reference / neutral pose, then press and release the hardware button to capture it.",
                CalibrationState.STEP_MAX: "Move slowly to the maximum safe position, fine-tune while watching the live reading, then press and release the hardware button.",
                CalibrationState.STEP_MIN: "Move slowly to the minimum safe position, fine-tune while watching the live reading, then press and release the hardware button.",
            }
            return f"{joint_name}: {step_prompts[self._calib.step]}"

        if self.mode == 1:
            return "Enable the joints you want to calibrate, choose CALIBRATE_JOINT, then use the hardware button to capture each point on release."

        return "Switch to CALIBRATION mode to begin a guided reference / max / min capture sequence."

    def _get_calibration_capture_summary(self) -> str:
        """Summarize captured values for the current joint during this calibration run."""
        if not self._calib.is_active:
            return "Captured values: none yet"

        captured = self._calib_session_values.get(self._calib.joint, {})
        return (
            "Captured values: "
            f"Ref {self._format_calibration_angle(captured.get('ref_raw'))}   "
            f"Max {self._format_calibration_angle(captured.get('max_raw'))}   "
            f"Min {self._format_calibration_angle(captured.get('min_raw'))}"
        )

    def _set_calibration_focus(self):
        """Highlight the joint currently being calibrated."""
        active_joint = self._calib.joint if self._calib.is_active else None
        step_label = None
        if self._calib.is_active:
            step_label = CalibrationState.STEP_LABELS[self._calib.step]

        for idx, box in enumerate(self.joint_boxes):
            box.set_calibration_focus(idx == active_joint, step_label if idx == active_joint else None)

    def _refresh_calibration_ui(self, current_raw=None, current_logical=None, button_state=None, status_override=None):
        """Refresh the calibration assistant panel and active-joint highlight."""
        if current_raw is not None:
            self._last_calib_raw = current_raw
        if current_logical is not None:
            self._last_calib_logical = current_logical
        if button_state is not None:
            self._last_calib_button_state = button_state

        self._set_calibration_focus()

        if self._calib.is_active:
            joint_idx = self._calib.joint
            step_label = CalibrationState.STEP_LABELS[self._calib.step]
            status_text = status_override or f"Active - Joint {joint_idx + 1} • Step {self._calib.step + 1}/3 • {step_label}"
            status_bg = "#1565c0"
            live_text = (
                "Live reading: "
                f"Raw {self._format_calibration_angle(self._last_calib_raw)}   "
                f"Logical {self._format_calibration_angle(self._last_calib_logical)}"
            )
            if self._last_calib_button_state is None:
                button_text = "Hardware button: waiting for telemetry"
            elif self._last_calib_button_state:
                button_text = "Hardware button: PRESSED - release now to capture"
            else:
                button_text = "Hardware button: ready - press and release when aligned"
        elif self.mode == 1:
            status_text = status_override or "Ready - calibration mode"
            status_bg = "#ef6c00"
            live_text = "Live reading: start calibration to lock onto one joint and watch it while measuring"
            button_text = "Hardware button: idle"
        else:
            status_text = status_override or "Calibration inactive"
            status_bg = "#616161"
            live_text = "Live reading: switch to CALIBRATION mode to begin"
            button_text = "Hardware button: unavailable outside calibration mode"

        self.calib_status_label.config(text=status_text, bg=status_bg)
        self.calib_instruction_var.set(self._get_calibration_instruction())
        self.calib_live_var.set(live_text)
        self.calib_button_var.set(button_text)
        self.calib_captures_var.set(self._get_calibration_capture_summary())

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
                # For out-of-range joints, hold current position instead of moving
                kwargs = {"current_mode": self.mode}
                logical_angles = []
                oor_joints = []
                for i, box in enumerate(self.joint_boxes):
                    if box.out_of_range and config.JOINTS[i].get("enabled", 0):
                        # Joint is out of range — hold current position
                        raw_angle = box.current_raw_angle
                        oor_joints.append(i + 1)
                    else:
                        _, logical_angle = box.get_state()
                        raw_angle = logical_to_raw(logical_angle, i)
                    kwargs[f"JOINT_{i+1}_ANG"] = round(raw_angle, 1)
                    logical_angles.append(f"J{i+1}={box.angle.get():.1f}°")
                if oor_joints:
                    oor_msg = f"Joints {oor_joints} out of range — holding position"
                    logger.warning(oor_msg)
                    debug_logger.log_event(oor_msg)
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
                # Start calibration from first enabled joint
                self._reset_calibration_session()
                started = self._calib.start(self.joint_boxes)
                if not started:
                    self._refresh_calibration_ui(status_override="No enabled joints selected")
                    messagebox.showwarning("Calibration", "Enable at least one calibratable joint before starting calibration.")
                    return
                self._refresh_calibration_ui(
                    status_override=f"Calibration started - Joint {self._calib.joint + 1} ready"
                )
                return  # No packet to send, just start listening
            
            elif selected_cmd == "GRIP_CNTL":
                # Build GRIP_CNTL command with slider value
                grip_angle = int(self.grip_var.get())
                packet = serial_protocol.build_packet(TYPE="CMD", CMD="GRIP_CNTL", current_mode=self.mode, GRIP_ANGLE=grip_angle)
                logger.info(f"Sending GRIP_CNTL: angle={grip_angle}°")
                
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
                    old_mode = self.mode
                    if old_mode == 1 and new_mode != 1:
                        if self._calib.is_active:
                            self._calib.stop()
                        self._reset_calibration_session()
                    # Stop any running preset if leaving MOVE mode
                    if new_mode != 2 and self._preset1_running:
                        self._stop_preset1()
                    self.mode = new_mode
                    self._telem_log_counter = 0  # Reset throttle on mode change to capture first telemetry
                    label_text = config.MODE_LABELS.get(self.mode, "UNKNOWN")
                    self.current_mode_label.config(text=f"MODE: {label_text.upper()}")
                    self._update_available_commands()
                    self._refresh_calibration_ui()
                    debug_logger.log_mode_change(old_mode, new_mode)
                    # Entering MOVE mode: queue a one-time slider sync so the first
                    # JOINTS_TO_ANGLE reflects actual encoder positions, not GUI defaults.
                    if new_mode == 2:  # MODE_MOVE
                        self._sync_sliders_on_next_telem = True
                
                self._log_received_packet(f"TYPE=ACK,CMD={selected_cmd} (verified)")
                debug_logger.log_ack(packet, verified=True)
                logger.info(f"Command {selected_cmd} acknowledged")
                
            except serial_protocol.ProtocolError as e:
                debug_logger.log_ack(packet, verified=False, error=str(e))
                messagebox.showerror("ACK Error", f"Failed to get ACK:\n{e}")
                logger.error(f"ACK error: {e}")
                
        except serial_protocol.ProtocolError as e:
            messagebox.showerror("Protocol Error", f"Failed to build packet:\n{str(e)}")
            debug_logger.log_error(f"Packet build error ({selected_cmd}): {e}")

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
                debug_logger.log_ack(packet, verified=True)
                debug_logger.log_event("ESTOP acknowledged by Teensy")
                messagebox.showwarning("ESTOP", "ESTOP command acknowledged by Teensy")
                logger.info("ESTOP acknowledged")
            except serial_protocol.ProtocolError as e:
                debug_logger.log_ack(packet, verified=False, error=str(e))
                debug_logger.log_error(f"ESTOP ACK failed: {e}")
                messagebox.showerror("ESTOP Error", f"ESTOP sent but no ACK received:\n{e}")
                logger.error(f"ESTOP ACK error: {e}")
                
        except serial_protocol.ProtocolError as e:
            messagebox.showerror("Protocol Error", f"Failed to build ESTOP packet:\n{str(e)}")

    def _on_grip_slider_changed(self, value):
        """Update grip value label when slider moves"""
        grip_angle = int(float(value))
        self.grip_value_label.config(text=f"{grip_angle}°")

    # =====================================================================
    # Inverse Kinematics Panel
    # =====================================================================

    def _create_ik_panel(self, parent):
        """Create the Inverse Kinematics control panel."""
        ik_frame = ttk.LabelFrame(parent, text="Inverse Kinematics")
        ik_frame.pack(side="left", fill="both", expand=True, padx=(5, 0))

        # Position inputs
        pos_frame = ttk.Frame(ik_frame)
        pos_frame.pack(fill="x", padx=8, pady=(8, 2))
        ttk.Label(pos_frame, text="Position (mm):", font=("TkDefaultFont", 9, "bold")).pack(anchor="w")

        pos_entries = ttk.Frame(pos_frame)
        pos_entries.pack(fill="x", pady=(2, 0))

        self.ik_x_var = tk.StringVar(value="243.7")
        self.ik_y_var = tk.StringVar(value="0.0")
        self.ik_z_var = tk.StringVar(value="412.4")

        for label, var in [("X:", self.ik_x_var), ("Y:", self.ik_y_var), ("Z:", self.ik_z_var)]:
            ttk.Label(pos_entries, text=label, width=3).pack(side="left")
            e = ttk.Entry(pos_entries, textvariable=var, width=8)
            e.pack(side="left", padx=(0, 8))
            e.bind("<Return>", lambda evt: self._on_ik_solve())

        # Orientation inputs
        ori_frame = ttk.Frame(ik_frame)
        ori_frame.pack(fill="x", padx=8, pady=2)
        ttk.Label(ori_frame, text="Orientation RPY (deg):", font=("TkDefaultFont", 9, "bold")).pack(anchor="w")

        ori_entries = ttk.Frame(ori_frame)
        ori_entries.pack(fill="x", pady=(2, 0))

        self.ik_rx_var = tk.StringVar(value="0.0")
        self.ik_ry_var = tk.StringVar(value="90.0")
        self.ik_rz_var = tk.StringVar(value="0.0")

        for label, var in [("Rx:", self.ik_rx_var), ("Ry:", self.ik_ry_var), ("Rz:", self.ik_rz_var)]:
            ttk.Label(ori_entries, text=label, width=3).pack(side="left")
            e = ttk.Entry(ori_entries, textvariable=var, width=8)
            e.pack(side="left", padx=(0, 8))
            e.bind("<Return>", lambda evt: self._on_ik_solve())

        # Solve & Send button
        self.ik_send_btn = tk.Button(
            ik_frame, text="Solve & Send",
            command=self._on_ik_solve,
            bg="#1565c0", fg="white",
            font=("TkDefaultFont", 10, "bold"),
            width=15
        )
        self.ik_send_btn.pack(pady=(8, 4))

        # Status label
        self.ik_status_var = tk.StringVar(value="Enter target pose, then Solve & Send")
        ttk.Label(
            ik_frame, textvariable=self.ik_status_var,
            wraplength=450, justify="left",
            font=("Courier", 9)
        ).pack(fill="x", padx=8, pady=(0, 2))

        # Result label (joint angles)
        self.ik_result_var = tk.StringVar(value="")
        ttk.Label(
            ik_frame, textvariable=self.ik_result_var,
            font=("Courier", 9)
        ).pack(fill="x", padx=8, pady=(0, 4))

        # ── Presets ──────────────────────────────────────────────────────
        preset_frame = ttk.LabelFrame(ik_frame, text="Presets")
        preset_frame.pack(fill="x", padx=8, pady=(2, 8))

        self.preset1_btn = tk.Button(
            preset_frame,
            text="▶  PRESET_1  (vertical sweep)",
            command=self._toggle_preset1,
            bg="#6a1b9a", fg="white",
            font=("TkDefaultFont", 10, "bold"),
            width=30,
        )
        self.preset1_btn.pack(side="left", padx=8, pady=6)

        self.preset1_status_var = tk.StringVar(value="Idle")
        ttk.Label(
            preset_frame, textvariable=self.preset1_status_var,
            font=("Courier", 9)
        ).pack(side="left", padx=8)

    def _on_ik_solve(self):
        """Compute IK and send the resulting joint angles to the arm."""
        # Must be in MOVE mode
        if self.mode != 2:
            self.ik_status_var.set("Must be in MOVE mode (mode 2)")
            self.ik_result_var.set("")
            return

        # Parse inputs
        try:
            x  = float(self.ik_x_var.get())
            y  = float(self.ik_y_var.get())
            z  = float(self.ik_z_var.get())
            rx = float(self.ik_rx_var.get())
            ry = float(self.ik_ry_var.get())
            rz = float(self.ik_rz_var.get())
        except ValueError:
            self.ik_status_var.set("Invalid input - enter numeric values")
            self.ik_result_var.set("")
            return

        self._send_ik_pose(x, y, z, rx, ry, rz)

    def _send_ik_pose(self, x: float, y: float, z: float,
                      rx: float, ry: float, rz: float) -> bool:
        """Solve IK for the given pose and send JOINTS_TO_ANGLE.

        Updates the IK status/result labels and GUI sliders.
        Returns True on success, False on IK failure or serial error.
        """
        # Solve IK
        try:
            result = ik_solver.solve_ik(x, y, z, rx, ry, rz)
        except ik_solver.IKError as e:
            self.ik_status_var.set(f"IK FAILED: {e}")
            self.ik_result_var.set("")
            return False

        angles = result['angles_deg']

        # Display computed joint angles
        parts = [f"J{i+1}={a:.1f}" for i, a in enumerate(angles)]
        self.ik_result_var.set("  ".join(parts))

        # Update sliders for enabled joints only
        for i, angle in enumerate(angles):
            if i < len(self.joint_boxes):
                jcfg = config.JOINTS[i] if i < len(config.JOINTS) else {}
                if jcfg.get("enabled", 0):
                    self.joint_boxes[i].angle.set(angle)
                    self.joint_boxes[i].val_label.config(text=f"{angle:.1f}\u00b0")

        # Build JOINTS_TO_ANGLE packet
        # Use IK angles for enabled joints, keep current slider for disabled
        # For out-of-range joints, hold current position
        kwargs = {"current_mode": self.mode}
        logical_parts = []
        oor_joints = []
        for i in range(6):
            jcfg = config.JOINTS[i] if i < len(config.JOINTS) else {}
            box = self.joint_boxes[i]
            if box.out_of_range and jcfg.get("enabled", 0):
                raw = box.current_raw_angle
                oor_joints.append(i + 1)
                logical_parts.append(f"J{i+1}=OOR")
            elif jcfg.get("enabled", 0) and i < len(angles):
                logical_angle = angles[i]
                raw = logical_to_raw(logical_angle, i)
                logical_parts.append(f"J{i+1}={logical_angle:.1f}\u00b0")
            else:
                _, logical_angle = self.joint_boxes[i].get_state()
                raw = logical_to_raw(logical_angle, i)
                logical_parts.append(f"J{i+1}={logical_angle:.1f}\u00b0")
            kwargs[f"JOINT_{i+1}_ANG"] = round(raw, 1)
        if oor_joints:
            oor_msg = f"IK: Joints {oor_joints} out of range — holding position"
            logger.warning(oor_msg)
            debug_logger.log_event(oor_msg)

        try:
            packet = serial_protocol.build_packet(TYPE="CMD", CMD="JOINTS_TO_ANGLE", **kwargs)
            self._log_sent_packet(packet)

            if self.serial_conn is None:
                self.ik_status_var.set("IK solved (offline - no serial)")
                return True  # Allow preset cycling in offline/dry-run mode

            serial_protocol.send_packet(self.serial_conn, packet)
            serial_protocol.wait_for_ack(packet, timeout=6.0)
            self._log_received_packet("TYPE=ACK,CMD=JOINTS_TO_ANGLE (IK)")
            debug_logger.log_ack(packet, verified=True)
            self.ik_status_var.set(f"Sent: {', '.join(logical_parts)}")
            logger.info(f"IK sent: {', '.join(logical_parts)}")
            return True

        except serial_protocol.ProtocolError as e:
            self.ik_status_var.set(f"IK solved but send failed: {e}")
            logger.error(f"IK send error: {e}")
            return False

    # =====================================================================
    # PRESET_1 — vertical pole sweep
    # =====================================================================

    def _toggle_preset1(self):
        """Start or stop PRESET_1."""
        if self._preset1_running:
            self._stop_preset1()
        else:
            self._start_preset1()

    def _start_preset1(self):
        if self.mode != 2:
            messagebox.showwarning("Preset", "Must be in MOVE mode to run presets.")
            return
        self._preset1_running = True
        self._preset1_step = 0  # Always begin at BOTTOM
        self.preset1_btn.config(text="■  Stop PRESET_1", bg="#b71c1c")
        debug_logger.log_event("PRESET_1 started — vertical pole sweep")
        self._preset1_tick()

    def _stop_preset1(self):
        self._preset1_running = False
        if self._preset1_after_id is not None:
            self.after_cancel(self._preset1_after_id)
            self._preset1_after_id = None
        self.preset1_btn.config(text="▶  PRESET_1  (vertical sweep)", bg="#6a1b9a")
        self.preset1_status_var.set("Stopped")
        debug_logger.log_event("PRESET_1 stopped")

    def _preset1_tick(self):
        """Execute one step of the PRESET_1 cycle and schedule the next."""
        if not self._preset1_running:
            return
        if self.mode != 2:
            self._stop_preset1()
            self.preset1_status_var.set("Stopped — mode changed")
            return

        name = _PRESET1_NAMES[self._preset1_step]
        pose = _PRESET1_POINTS[self._preset1_step]
        self.preset1_status_var.set(f"Moving → {name}…")
        self.update_idletasks()  # Refresh label before blocking send

        ok = self._send_ik_pose(*pose)
        if not ok:
            self._stop_preset1()
            self.preset1_status_var.set("Stopped — IK / send error")
            return

        # Advance to next step and schedule next tick
        self._preset1_step = 1 - self._preset1_step
        next_name = _PRESET1_NAMES[self._preset1_step]
        delay_s = _PRESET1_DELAY_MS // 1000
        self.preset1_status_var.set(
            f"At {name} — next: {next_name} in {delay_s}s"
        )
        self._preset1_after_id = self.after(_PRESET1_DELAY_MS, self._preset1_tick)

    def _on_close(self):
        """Clean shutdown on window close"""
        try:
            serial_protocol.stop_listener()
            if self.serial_conn is not None:
                self.serial_conn.close()
                logger.info("Serial connection closed")
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
        debug_logger.log_event("Application closed")
        debug_logger.close()
        self.destroy()


def main():
    app = ArmGUI()
    app.mainloop()


if __name__ == "__main__":
    main()
