# Configuration for the Tkinter GUI

# Serial port configuration (adjust for your system)
SERIAL_PORT = {
    "PORT": "COM3",
    "BAUD": 115200,
    # If you want the GUI to only print packets instead of sending,
    # set DRY_RUN = True
    "DRY_RUN": False,
}

# Default mode and labels
# 0=Idle,1=Calibration,2=Move,3=Reserved
DEFAULT_MODE = 0


# Number of joints shown in the GUI (6 expected)
NUM_JOINTS = 6

# Per-joint configuration: label, min_angle, max_angle, start_angle
# Edit these values to change slider ranges and labels.
JOINTS = [
    {"label": "Joint 1", "min": 0.0, "max": 90.0, "start": 0.0, "enabled": 1},
    {"label": "Joint 2 shoulder", "min": 0.0, "max": 90.0, "start": 0.0, "enabled": 0},
    {"label": "Joint 3", "min": 0.0, "max": 90.0, "start": 0.0, "enabled": 1},
    {"label": "Joint 4", "min": 0.0, "max": 180.0, "start": 0.0, "enabled": 0},
    {"label": "Joint 5", "min": 0.0, "max": 180.0, "start": 0.0, "enabled": 0},
    {"label": "Joint 6", "min": 0.0, "max": 180.0, "start": 0.0, "enabled": 0},
]

# ============================================================================
# SERIAL PROTOCOL: Hierarchical Schema Definition
# ============================================================================
# Structure: TYPE -> COMMAND -> allowed_modes -> key constraints
#
# - TYPE: CMD (UI→Teensy), DATA (Teensy→UI), ACK (Teensy→UI, echoes CMD)
# - COMMAND: Specific action (e.g., SET_MODE, JOINTS_TO_ANGLE)
# - allowed_modes: List of modes where this command is valid (0=IDLE, 1=CALIB, 2=MOVE, 3=RESERVED)
# - required_keys/optional_keys: Key names that must/may be present
# - key_constraints: Dict mapping key names to allowed values (if restricted)
#
# ACK Protocol:
# - Teensy echoes the original CMD packet back EXACTLY, changing only TYPE=CMD to TYPE=ACK
# - If ACK does not match original CMD (except TYPE), UI displays error popup
# - This provides verification without needing sequence numbers or checksums

PROTOCOL_SCHEMAS = {
    "CMD": {
        "SET_MODE": {
            "allowed_modes": [0, 1, 2, 3],  # Can send from any mode
            "required_keys": ["MODE"],
            "optional_keys": [],
            "key_constraints": {
                "MODE": [0, 1, 2, 3]  # Idle, Calibration, Move, Reserved
            }
        },
        "JOINTS_TO_ANGLE": {
            "allowed_modes": [2],  # Only in MOVE mode
            "required_keys": ["JOINT_1_ANG", "JOINT_2_ANG", "JOINT_3_ANG", 
                              "JOINT_4_ANG", "JOINT_5_ANG", "JOINT_6_ANG"],
            "optional_keys": [],
            "key_constraints": {}  # Float values, range checked at Teensy
        },
        "JOINT_EN": {
            "allowed_modes": [2],  # Can send from any mode
            "required_keys": ["JOINT_1_EN", "JOINT_2_EN", "JOINT_3_EN",
                              "JOINT_4_EN", "JOINT_5_EN", "JOINT_6_EN"],
            "optional_keys": [],
            "key_constraints": {
                "JOINT_1_EN": [0, 1],
                "JOINT_2_EN": [0, 1],
                "JOINT_3_EN": [0, 1],
                "JOINT_4_EN": [0, 1],
                "JOINT_5_EN": [0, 1],
                "JOINT_6_EN": [0, 1]
            }
        },
        "ESTOP": {
            "allowed_modes": [2],  # Only in MOVE mode
            "required_keys": ["STOP"],
            "optional_keys": [],
            "key_constraints": {
                "STOP": ["ALL"]
            }
        },
        "CALIBRATE_JOINT": {
            "allowed_modes": [1],  # Only in CALIBRATION mode
            "required_keys": ["JOINT_ID"],
            "optional_keys": [],
            "key_constraints": {
                "JOINT_ID": [1, 2, 3, 4, 5, 6]
            }
        }
    },
    "DATA": {
        "JOINT_ANGLES": {
            "allowed_modes": [1, 2],  # Calibration and Move modes
            "required_keys": ["ENCODER_1_ANGLE", "ENCODER_2_ANGLE", "ENCODER_3_ANGLE",
                              "ENCODER_4_ANGLE", "ENCODER_5_ANGLE", "ENCODER_6_ANGLE",
                              "BUTTON"],
            "optional_keys": [],
            "key_constraints": {
                "BUTTON": [0, 1]  # 0=not pressed, 1=pressed
            }
        }
    },
    "ACK": {
        # ACK echoes the original CMD packet exactly (only TYPE changes)
        # No schema validation needed; just verify original CMD was valid
        # Format: TYPE=ACK,CMD=<same_as_original>,...<same_fields_as_original>
    }
}

# Mode definitions
MODE_LABELS = {
    0: "IDLE",
    1: "CALIBRATION",
    2: "MOVE",
    3: "RESERVED"
}

# Packet format settings (legacy, to be deprecated)
# We will send a single CSV line: ESTOP, j0_enable, j0_pos, j1_enable, j1_pos, ...\n