# Configuration for the Tkinter GUI

# Serial port configuration (adjust for your system)
SERIAL_PORT = {
    "PORT": "COM3",
    "BAUD": 115200,
    # If you want the GUI to only print packets instead of sending,
    # set DRY_RUN = True
    "DRY_RUN": False,
}

# Number of joints shown in the GUI (6 expected)
NUM_JOINTS = 6

# Per-joint configuration: label, min_angle, max_angle, start_angle
# Edit these values to change slider ranges and labels.
JOINTS = [
    {"label": "Joint 1", "min": 0.0, "max": 90.0, "start": 0.0, "enabled": 0},
    {"label": "Joint 2 shoulder", "min": 0.0, "max": 90.0, "start": 0.0, "enabled": 0},
    {"label": "Joint 3", "min": 0.0, "max": 90.0, "start": 0.0, "enabled": 0},
    {"label": "Joint 4", "min": 0.0, "max": 180.0, "start": 0.0, "enabled": 0},
    {"label": "Joint 5", "min": 0.0, "max": 180.0, "start": 0.0, "enabled": 0},
    {"label": "Joint 6", "min": 0.0, "max": 180.0, "start": 0.0, "enabled": 0},
]

# Packet format settings
# We will send a single CSV line: ESTOP, j0_enable, j0_pos, j1_enable, j1_pos, ...\n