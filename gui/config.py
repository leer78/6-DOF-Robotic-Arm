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

# Per-joint configuration with Raw-to-Logical mapping
# 
# CALIBRATION VALUES (captured during calibration or set manually):
#   ref_raw:    Raw encoder value at neutral/reference position
#   ref_offset: Logical angle that ref_raw represents (your design choice, e.g., 90°)
#   direction:  +1 = same direction, -1 = inverted (if CW increases raw but should decrease logical)
#   min_raw:    Raw encoder value at minimum logical position
#   max_raw:    Raw encoder value at maximum logical position
#
# DERIVED VALUES (computed dynamically, not stored):
#   min_deg = (min_raw - ref_raw) * direction + ref_offset
#   max_deg = (max_raw - ref_raw) * direction + ref_offset
#
# Mapping formulas:
#   logical = (raw - ref_raw) * direction + ref_offset
#   raw = (logical - ref_offset) / direction + ref_raw
#
JOINTS = [
    {"label": "Joint 1", "enabled": 0,        # No encoder — disabled (not part of RRR)
     "ref_raw": 0.0, "ref_offset": 0.0, "direction": 1,
     "min_raw": 0.0, "max_raw": 90.0},
    {"label": "Joint 2 shoulder", "enabled": 1,  # AS5600 wired — RRR joint 1
     "ref_raw": 301.9, "ref_offset": 0.0, "direction": -1,
     "min_raw": 326.2, "max_raw": 280.5},
    {"label": "Joint 3", "enabled": 1,            # AS5600 wired — RRR joint 2
     "ref_raw": 203.4, "ref_offset": 0.0, "direction": -1,
     "min_raw": 229.3, "max_raw": 114.0},
    {"label": "Joint 4", "enabled": 1,            # AS5600 wired — wrist pitch
     "ref_raw": 103.0, "ref_offset": 0.0, "direction": -1,
     "min_raw": 50.0, "max_raw": 150.0},
    {"label": "Joint 5", "enabled": 1,            # AS5600 wired — RRR joint 3
     "ref_raw": 83.8, "ref_offset": 0.0, "direction": -1,
     "min_raw": 169.2, "max_raw": 357.5},
    {"label": "Joint 6", "enabled": 0,        # No encoder — disabled (not part of RRR)
     "ref_raw": 120.0, "ref_offset": 120.0, "direction": 1,
     "min_raw": 60.0, "max_raw": 180.0},
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
        },
        "GRIP_CNTL": {
            "allowed_modes": [2],  # Only in MOVE mode
            "required_keys": ["GRIP_ANGLE"],
            "optional_keys": [],
            "key_constraints": {}  # Float value 0-180, range checked at Teensy
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
        },
        "PID_DEBUG": {
            "allowed_modes": [2],  # Only in MOVE mode
            "required_keys": [],   # Dynamic keys per enabled joint (J2_ERR, J2_CMD, etc.)
            "optional_keys": [],
            "key_constraints": {}
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
