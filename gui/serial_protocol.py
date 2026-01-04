"""
Serial Protocol Module - Two-Lane Highway Communication

Handles packet building, parsing, and validation for UI ↔ Teensy communication.
Lane 1: Commands (UI → Teensy) + Acknowledgments (Teensy → UI)
Lane 2: Telemetry (Teensy → UI, fire-and-forget)
"""

import config
from config import PROTOCOL_SCHEMAS, MODE_LABELS
from typing import Dict, Any, Optional, Callable
import threading
import time
import logging

logger = logging.getLogger(__name__)


class ProtocolError(Exception):
    """Raised when packet validation fails."""
    pass


# ============================================================================
# LISTENER GLOBALS AND SYNCHRONIZATION
# ============================================================================

current_ack: Optional[str] = None
current_ack_lock = threading.Lock()

_listener_thread: Optional[threading.Thread] = None
_listener_stop_flag = threading.Event()
_listener_serial_conn = None
_telemetry_handler: Optional[Callable[[str], None]] = None

# Overview:
# - Two logical lanes: command/ACK (synchronous) and telemetry (asynchronous).
# - `current_ack` holds the latest ACK line; protect access with `current_ack_lock`.
# - Telemetry (TYPE=DATA) is dispatched to `_telemetry_handler` if set.

# ============================================================================
# SERIAL CONNECTION MANAGEMENT
# ============================================================================

def connect_serial():
    """
    Open a serial connection to the Teensy using config.SERIAL_PORT settings.

    Returns:
        serial.Serial: Open serial connection object

    Raises:
        ProtocolError: If connection fails
        
    Example:
        >>> conn = connect_serial()
        >>> conn.write(b"TYPE=CMD,CMD=SET_MODE,MODE=2\\n")
        >>> conn.close()
    """
    import serial
    import time
    
    port = config.SERIAL_PORT["PORT"]
    baud = config.SERIAL_PORT["BAUD"]
    
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # Wait for Teensy to boot and initialize
        return ser
    except serial.SerialException as e:
        raise ProtocolError(f"Failed to open serial port {port}@{baud}: {str(e)}")
    except Exception as e:
        raise ProtocolError(f"Unexpected error connecting to {port}: {str(e)}")

def build_packet(TYPE: str, CMD: str = None, current_mode: int = None, **kwargs) -> str:
    """
    Build and validate a serial protocol packet.

    Args:
        TYPE: Packet type ('CMD', 'DATA', 'ACK')
        CMD: Command/data type name (e.g., 'SET_MODE', 'JOINTS_TO_ANGLE')
        current_mode: Current operating mode (required for MODE validation)
        **kwargs: Key-value pairs for packet fields (e.g., MODE=2, JOINT_1_ANGLE=45.5)

    Returns:
        Formatted packet string: "TYPE=<type>,KEY1=VALUE1,KEY2=VALUE2,...\n"

    Raises:
        ProtocolError: If validation fails (missing keys, invalid values, etc.)

    Examples:
        >>> build_packet(TYPE='CMD', CMD='SET_MODE', MODE=2)
        'TYPE=CMD,CMD=SET_MODE,MODE=2\n'

        >>> build_packet(TYPE='CMD', CMD='JOINTS_TO_ANGLE', current_mode=2,
        ...              JOINT_1_ANGLE=45.5, JOINT_2_ANGLE=67.2, ...)
        'TYPE=CMD,CMD=JOINTS_TO_ANGLE,JOINT_1_ANGLE=45.5,...\n'
    """

    # Validation summary:
    # 1) Ensure TYPE exists in PROTOCOL_SCHEMAS
    # 2) For CMD/DATA: ensure CMD is in schema, required keys present,
    #    and values/modes are valid
    # 3) Produce deterministic packet string ending with '\n'

    # Validate TYPE
    if TYPE not in PROTOCOL_SCHEMAS:
        raise ProtocolError(f"Invalid TYPE: {TYPE}. Must be one of: {list(PROTOCOL_SCHEMAS.keys())}")

    # For CMD/DATA types, validate CMD exists in schema
    if TYPE in ["CMD", "DATA"]:
        if CMD is None:
            raise ProtocolError(f"CMD parameter required for TYPE={TYPE}")

        schema_dict = PROTOCOL_SCHEMAS[TYPE]
        if CMD not in schema_dict:
            raise ProtocolError(f"Unknown {TYPE} command: {CMD}. Valid commands: {list(schema_dict.keys())}")

        command_schema = schema_dict[CMD]

        # Check mode restrictions
        allowed_modes = command_schema.get("allowed_modes", [])
        if current_mode is not None and current_mode not in allowed_modes:
            mode_names = [MODE_LABELS.get(m, str(m)) for m in allowed_modes]
            raise ProtocolError(
                f"Command '{CMD}' not allowed in mode {current_mode} ({MODE_LABELS.get(current_mode, 'UNKNOWN')}). "
                f"Allowed modes: {mode_names}"
            )

        # Validate required keys are present
        required_keys = command_schema.get("required_keys", [])
        provided_keys = set(kwargs.keys())
        missing_keys = set(required_keys) - provided_keys

        if missing_keys:
            raise ProtocolError(
                f"Command '{CMD}' missing required keys: {list(missing_keys)}. "
                f"Required: {required_keys}"
            )

        # Validate no unexpected keys
        allowed_keys = set(required_keys) | set(command_schema.get("optional_keys", []))
        unexpected_keys = provided_keys - allowed_keys

        if unexpected_keys:
            raise ProtocolError(
                f"Command '{CMD}' received unexpected keys: {list(unexpected_keys)}. "
                f"Allowed: {list(allowed_keys)}"
            )

        # Validate key constraints (if specified)
        key_constraints = command_schema.get("key_constraints", {})
        for key, constraint_values in key_constraints.items():
            if key in kwargs:
                value = kwargs[key]
                # Convert constraint values to strings for comparison if needed
                constraint_values_str = [str(v) for v in constraint_values]
                if str(value) not in constraint_values_str:
                    raise ProtocolError(
                        f"Key '{key}' = {value} not in allowed values: {constraint_values}"
                    )

    elif TYPE == "ACK":
        # ACK just echoes CMD, no validation needed here
        if CMD is None:
            raise ProtocolError("CMD parameter required for TYPE=ACK")

    # Build packet: TYPE first, then CMD, then other key-value pairs in order
    packet_parts = [f"TYPE={TYPE}"]

    if CMD is not None:
        packet_parts.append(f"CMD={CMD}")

    # Add remaining kwargs in consistent order (sorted for deterministic output)
    for key in sorted(kwargs.keys()):
        value = kwargs[key]
        packet_parts.append(f"{key}={value}")

    packet_str = ",".join(packet_parts) + "\n"

    return packet_str


def parse_packet(packet_str: str) -> Dict[str, Any]:
    """
    Parse a received packet string into a dictionary.

    Args:
        packet_str: Raw packet string (e.g., "TYPE=CMD,CMD=SET_MODE,MODE=2\n")

    Returns:
        Dictionary with parsed keys and values

    Raises:
        ProtocolError: If packet is malformed

    Example:
        >>> parse_packet("TYPE=CMD,CMD=SET_MODE,MODE=2\n")
        {'TYPE': 'CMD', 'CMD': 'SET_MODE', 'MODE': '2'}
    """

    # Trim whitespace/newlines and ensure input is not empty
    packet_str = packet_str.strip()

    if not packet_str:
        raise ProtocolError("Empty packet")

    try:
        pairs = packet_str.split(",")
        parsed = {}

        for pair in pairs:
            pair = pair.strip()
            if not pair:
                continue
                
            if "=" not in pair:
                raise ProtocolError(f"Malformed key-value pair: {pair}")

            key, value = pair.split("=", 1)
            key = key.strip()
            value = value.strip()

            if not key:
                raise ProtocolError(f"Empty key in pair: {pair}")

            parsed[key] = value

        if "TYPE" not in parsed:
            raise ProtocolError("Packet missing TYPE field")

        return parsed

    except ProtocolError:
        raise
    except Exception as e:
        raise ProtocolError(f"Failed to parse packet: {str(e)}")


def verify_ack(sent_packet_str: str, received_ack_str: str) -> bool:
    """
    Verify that received ACK matches the sent CMD (except for TYPE field).

    Args:
        sent_packet_str: Original CMD packet that was sent
        received_ack_str: Received ACK packet to verify

    Returns:
        True if ACK matches CMD (TYPE field may differ)

    Raises:
        ProtocolError: If ACK structure doesn't match

    Example:
        >>> sent = "TYPE=CMD,CMD=SET_MODE,MODE=2\n"
        >>> ack = "TYPE=ACK,CMD=SET_MODE,MODE=2\n"
        >>> verify_ack(sent, ack)
        True
    """

    # Parse and compare all fields except TYPE to verify ACK content
    sent_parsed = parse_packet(sent_packet_str.strip())
    ack_parsed = parse_packet(received_ack_str.strip())

    # Check that ACK TYPE is correct
    if ack_parsed.get("TYPE") != "ACK":
        raise ProtocolError(f"Expected TYPE=ACK, got TYPE={ack_parsed.get('TYPE')}")

    # Compare all fields except TYPE
    sent_without_type = {k: v for k, v in sent_parsed.items() if k != "TYPE"}
    ack_without_type = {k: v for k, v in ack_parsed.items() if k != "TYPE"}

    if sent_without_type != ack_without_type:
        raise ProtocolError(
            f"ACK mismatch:\n  Sent: {sent_without_type}\n  Received: {ack_without_type}"
        )

    return True


# ============================================================================
# LISTENER THREAD AND ACK HANDLING
# ============================================================================

def set_telemetry_handler(handler: Callable[[str], None]) -> None:
    """Set a callback for incoming DATA packets (telemetry)."""
    global _telemetry_handler
    _telemetry_handler = handler
    # Note: handler is invoked from the listener thread; keep it thread-safe


def start_listener(serial_conn) -> None:
    """Start the background listener thread that parses incoming packets."""
    global _listener_thread, _listener_serial_conn, current_ack
    
    # Prevent duplicate listeners
    if _listener_thread is not None and _listener_thread.is_alive():
        logger.warning("Listener already running")
        return
    
    # Store the connection object the listener will read from
    _listener_serial_conn = serial_conn
    _listener_stop_flag.clear()

    # Clear any previous ACK state before starting
    with current_ack_lock:
        current_ack = None

    # Start background thread (daemon so it won't block process exit)
    _listener_thread = threading.Thread(target=_listener_thread_packet, daemon=True)
    _listener_thread.start()
    logger.info("Listener thread started")


def stop_listener() -> None:
    """Stop the background listener thread cleanly."""
    global _listener_thread, _listener_serial_conn
    # Signal the listener to stop and try a short join for clean shutdown
    _listener_stop_flag.set()
    if _listener_thread is not None:
        _listener_thread.join(timeout=2.0)
        if _listener_thread.is_alive():
            logger.warning("Listener thread did not stop within timeout")
        _listener_thread = None
    # Clear stored serial connection reference
    _listener_serial_conn = None
    logger.info("Listener thread stopped")


def _listener_thread_packet() -> None:
    """Internal loop that reads serial data, parses packets, and routes ACK/DATA."""
    global current_ack
    
    buffer = b""
    
    while not _listener_stop_flag.is_set():
        try:
            if _listener_serial_conn is None:
                time.sleep(0.1)
                continue
            
            # Check if data is available
            in_waiting = getattr(_listener_serial_conn, 'in_waiting', 0)
            if in_waiting > 0: #data is available
                chunk = _listener_serial_conn.read(in_waiting)
                if chunk:
                    buffer += chunk
            else:
                # No data available, sleep briefly to avoid busy-waiting
                time.sleep(0.01)
                continue
            
            # Process complete lines
            while b'\n' in buffer:
                line_bytes, buffer = buffer.split(b'\n', 1)
                try:
                    line = line_bytes.decode('utf-8').strip()
                except UnicodeDecodeError:
                    logger.warning(f"Failed to decode line: {line_bytes!r}")
                    continue
                
                if not line:
                    continue
                
                try:
                    parsed = parse_packet(line)
                    ptype = parsed.get("TYPE") #currently, only two types would be seen from teensy: ACK and DATA
                    
                    if ptype == "ACK":
                        with current_ack_lock:
                            current_ack = line
                        logger.debug(f"ACK received: {line}")
                    elif ptype == "DATA":
                        if _telemetry_handler is not None:
                            try:
                                _telemetry_handler(line)
                            except Exception as e:
                                logger.error(f"Telemetry handler error: {e}")
                        logger.debug(f"DATA received: {line}")
                    else:
                        logger.debug(f"Other packet received: {line}")
                        
                except ProtocolError as e:
                    logger.warning(f"Malformed packet: {line} - {e}")
                    
        except Exception as e:
            logger.error(f"Listener read error: {e}")
            time.sleep(0.1)


def send_packet(serial_conn, packet_str: str) -> None:
    """Write a packet to the serial connection, ensuring newline and flush."""
    # Ensure a single trailing newline; convert to bytes and write.
    if not packet_str.endswith('\n'):
        packet_str = packet_str + '\n'

    try:
        serial_conn.write(packet_str.encode('utf-8'))
        # Flush if available to minimize latency
        if hasattr(serial_conn, 'flush'):
            serial_conn.flush()
    except Exception as e:
        # Surface errors as ProtocolError for callers
        raise ProtocolError(f"Failed to send packet: {e}")


def wait_for_ack(sent_packet: str, timeout: float = 5.0) -> bool:
    """Block until a matching ACK is received or timeout expires."""
    global current_ack
    
    start = time.time()
    
    # Poll loop: check `current_ack` under lock, attempt verification
    # on any observed ACK; if verification succeeds consume it and
    # return True. If ACK present but doesn't match, keep waiting
    # (policy: do not discard unmatched ACKs here).
    while time.time() - start < timeout:
        with current_ack_lock:
            ack = current_ack

        if ack is not None:
            try:
                # verify_ack expects newline-terminated strings
                sent_with_newline = sent_packet if sent_packet.endswith('\n') else sent_packet + '\n'
                ack_with_newline = ack + '\n'
                verify_ack(sent_with_newline, ack_with_newline)

                # ACK verified - consume it and return
                with current_ack_lock:
                    current_ack = None
                return True
            except ProtocolError:
                # not a matching ACK; continue waiting
                pass

        # Small sleep reduces CPU usage while keeping latency low
        time.sleep(0.01)

    # Timeout expired without matching ACK
    raise ProtocolError(f"Timeout waiting for ACK (sent: {sent_packet.strip()})")


