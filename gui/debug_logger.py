"""
Debug Logger Module

Creates a timestamped log file under 6-DOF-Robotic-Arm/debug_logs/ on each
app startup. Logs every packet sent/received, joint positions during MOVE and
CALIBRATION modes, mode transitions, calibration events, and errors.

Usage:
    import debug_logger
    debug_logger.init()            # Call once at startup
    debug_logger.log_sent(packet)
    debug_logger.log_received(packet)
    debug_logger.log_joint_positions(mode, raw_angles, logical_angles)
    debug_logger.log_mode_change(old_mode, new_mode)
    debug_logger.log_event(message)
    debug_logger.log_error(message)
    debug_logger.close()
"""

import os
import sys
from datetime import datetime

# ============================================================================
# MODULE STATE
# ============================================================================

_log_file = None       # Open file handle
_log_path = None       # Full path to current log file
_session_start = None  # datetime of session start


def init():
    """
    Open a new timestamped log file under debug_logs/.

    Creates the directory if it doesn't exist. Writes a session header
    with startup configuration (serial port, joint config, Python version).
    Safe to call multiple times — only opens a file on the first call.
    """
    global _log_file, _log_path, _session_start

    if _log_file is not None:
        return  # Already initialized

    # Resolve debug_logs/ relative to this file's parent (repo root)
    gui_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(gui_dir)
    logs_dir = os.path.join(repo_root, "debug_logs")

    os.makedirs(logs_dir, exist_ok=True)

    _session_start = datetime.now()
    filename = _session_start.strftime("%Y%m%d_%H%M%S") + "_log.txt"
    _log_path = os.path.join(logs_dir, filename)

    _log_file = open(_log_path, "w", encoding="utf-8", buffering=1)  # line-buffered

    _write_header()
    print(f"[debug_logger] Logging to: {_log_path}")


def _write_header():
    """Write session header with system and config info."""
    _writeln("=" * 72)
    _writeln(f"6-DOF ROBOTIC ARM -- DEBUG SESSION LOG")
    _writeln(f"Started : {_session_start.strftime('%Y-%m-%d %H:%M:%S')}")
    _writeln(f"Python  : {sys.version.split()[0]}  ({sys.platform})")
    _writeln(f"Log file: {_log_path}")
    _writeln("=" * 72)

    # Dump serial config
    try:
        import config
        sp = config.SERIAL_PORT
        _writeln(f"\n[CONFIG] Serial port : {sp.get('PORT', '?')} @ {sp.get('BAUD', '?')} baud")
        _writeln(f"[CONFIG] Dry-run mode: {sp.get('DRY_RUN', False)}")
        _writeln(f"[CONFIG] Default mode: {config.DEFAULT_MODE} "
                 f"({config.MODE_LABELS.get(config.DEFAULT_MODE, 'UNKNOWN')})")
        _writeln(f"[CONFIG] Num joints  : {config.NUM_JOINTS}")

        _writeln("\n[CONFIG] Joint configuration:")
        _writeln(f"  {'#':<4} {'Label':<22} {'Enabled':<9} "
                 f"{'ref_raw':>8} {'ref_off':>8} {'dir':>4} "
                 f"{'min_raw':>8} {'max_raw':>8}")
        _writeln(f"  {'-'*4} {'-'*22} {'-'*9} "
                 f"{'-'*8} {'-'*8} {'-'*4} {'-'*8} {'-'*8}")
        for idx, j in enumerate(config.JOINTS):
            _writeln(
                f"  J{idx+1:<3} {j.get('label',''):<22} "
                f"{'YES' if j.get('enabled') else 'no':<9} "
                f"{j.get('ref_raw', 0):>8.1f} "
                f"{j.get('ref_offset', 0):>8.1f} "
                f"{j.get('direction', 1):>4} "
                f"{j.get('min_raw', 0):>8.1f} "
                f"{j.get('max_raw', 0):>8.1f}"
            )
    except Exception as e:
        _writeln(f"[CONFIG] (could not load config: {e})")

    _writeln("\n" + "=" * 72)
    _writeln("TIMELINE")
    _writeln("=" * 72)


# ============================================================================
# PUBLIC LOGGING API
# ============================================================================

def log_event(message: str):
    """Log a general informational event."""
    _writeln(f"[{_ts()}] [EVENT] {message}")


def log_error(message: str):
    """Log an error or warning."""
    _writeln(f"[{_ts()}] [ERROR] {message}")


def log_sent(packet: str):
    """Log a packet sent from the GUI to the Teensy."""
    _writeln(f"[{_ts()}] [SEND ] {packet.strip()}")


def log_received(packet: str):
    """Log a packet received from the Teensy."""
    _writeln(f"[{_ts()}] [RECV ] {packet.strip()}")


def log_mode_change(old_mode: int, new_mode: int):
    """Log a mode transition."""
    try:
        import config
        old_label = config.MODE_LABELS.get(old_mode, "UNKNOWN")
        new_label = config.MODE_LABELS.get(new_mode, "UNKNOWN")
    except Exception:
        old_label = str(old_mode)
        new_label = str(new_mode)
    _writeln(f"[{_ts()}] [MODE ] {old_mode} ({old_label}) -> {new_mode} ({new_label})")


def log_joint_positions(mode: int, raw_angles: list, logical_angles: list,
                        enabled: list = None, button: int = None):
    """
    Log all joint positions.

    Args:
        mode: Current operating mode integer
        raw_angles: List of 6 raw encoder floats
        logical_angles: List of 6 logical (degree) floats
        enabled: Optional list of 6 booleans indicating enabled joints
        button: Optional button state (0 or 1)
    """
    try:
        import config
        mode_label = config.MODE_LABELS.get(mode, "?")
    except Exception:
        mode_label = str(mode)

    parts = [f"[{_ts()}] [JPOS ] mode={mode}({mode_label})"]

    headers = []
    raw_vals = []
    logical_vals = []
    for i in range(6):
        en_tag = ""
        if enabled is not None and i < len(enabled):
            en_tag = "*" if enabled[i] else " "
        else:
            en_tag = " "

        raw_val = raw_angles[i] if i < len(raw_angles) else float("nan")
        log_val = logical_angles[i] if i < len(logical_angles) else float("nan")
        headers.append(f"J{i+1}{en_tag}")
        raw_vals.append(f"{raw_val:>8.2f}")
        logical_vals.append(f"{log_val:>8.2f}")

    col_w = 9
    hdr_str  = "  " + "".join(f"{h:>{col_w}}" for h in headers)
    raw_str  = "  " + "  raw(deg):" + "".join(f"{v}" for v in raw_vals)
    log_str  = "  " + "  log(deg):" + "".join(f"{v}" for v in logical_vals)

    _writeln("\n".join(parts))
    _writeln(hdr_str)
    _writeln(raw_str)
    _writeln(log_str)

    if button is not None:
        _writeln(f"         button={button}")


def log_calibration_capture(joint_idx: int, step_label: str,
                             raw_angle: float, step_num: int):
    """Log a calibration capture event."""
    _writeln(
        f"[{_ts()}] [CALIB] Joint {joint_idx+1}, step {step_num}/3 ({step_label}): "
        f"raw={raw_angle:.2f}°"
    )


def log_calibration_complete(joint_idx: int, ref_raw: float, min_raw: float,
                              max_raw: float, min_logical: float, max_logical: float):
    """Log the computed limits after a joint's calibration is finalized."""
    _writeln(
        f"[{_ts()}] [CALIB] Joint {joint_idx+1} calibration COMPLETE  "
        f"ref_raw={ref_raw:.2f}  min_raw={min_raw:.2f}  max_raw={max_raw:.2f}  "
        f"-> logical min={min_logical:.2f} deg  max={max_logical:.2f} deg"
    )


def log_serial_connect(port: str, baud: int, success: bool, error: str = None):
    """Log serial connection attempt result."""
    if success:
        _writeln(f"[{_ts()}] [SERIAL] Connected: {port} @ {baud} baud")
    else:
        _writeln(f"[{_ts()}] [SERIAL] FAILED to connect: {port} @ {baud} — {error}")


def log_ack(packet: str, verified: bool, error: str = None):
    """Log ACK verification result."""
    if verified:
        _writeln(f"[{_ts()}] [ACK  ] VERIFIED for: {packet.strip()}")
    else:
        _writeln(f"[{_ts()}] [ACK  ] FAILED for: {packet.strip()} — {error}")


def close():
    """Flush and close the log file, writing a session footer."""
    global _log_file
    if _log_file is None:
        return
    duration = datetime.now() - _session_start
    _writeln("\n" + "=" * 72)
    _writeln(f"SESSION ENDED -- duration: {duration}")
    _writeln("=" * 72 + "\n")
    try:
        _log_file.flush()
        _log_file.close()
    except Exception:
        pass
    _log_file = None


# ============================================================================
# INTERNAL HELPERS
# ============================================================================

def _ts() -> str:
    """Return current time as HH:MM:SS.mmm string."""
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def _writeln(line: str):
    """Write a line to the log file (no-op if not initialized)."""
    if _log_file is None:
        return
    try:
        _log_file.write(line + "\n")
    except Exception:
        pass
