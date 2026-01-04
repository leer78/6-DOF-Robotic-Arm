# Serial Communication Protocol Design
## 6-DOF Robotic Arm: UI ↔ Teensy 4.1

**Status:** Design Phase  
**Baud Rate:** 115200

Two independent logical channels over one physical serial link:
- **Lane 1 (CMD):** UI → Teensy (requires ACK/NAK)
- **Lane 2 (DATA):** Teensy → UI (fire-and-forget)

---

## Implementation: `serial_protocol.py`

### `build_packet()` Function

**Purpose:** Build and validate packets before sending. Ensures all constraints from `PROTOCOL_SCHEMAS` are met.

**Signature:**
```python
def build_packet(TYPE: str, CMD: str = None, current_mode: int = None, **kwargs) -> str
```

**Parameters:**
- `TYPE`: 'CMD', 'DATA', or 'ACK'
- `CMD`: Command/data type name (e.g., 'SET_MODE', 'JOINTS_TO_ANGLE')
- `current_mode`: Current operating mode (used for mode-restriction validation)
- `**kwargs`: Key-value pairs (e.g., MODE=2, JOINT_1_ANGLE=45.5)

**Returns:** Formatted packet string with newline terminator
```
TYPE=<type>,KEY1=VALUE1,KEY2=VALUE2,...\n
```

**Validations Performed:**
1. TYPE is valid (CMD, DATA, ACK)
2. CMD exists in schema
3. Current mode is in allowed_modes list
4. All required keys are present
5. No unexpected keys are provided
6. Key values match constraints (if defined)

**Raises:** `ProtocolError` with descriptive message on any validation failure

**Usage Examples:**

```python
# Set mode to MOVE (mode 2)
build_packet(TYPE='CMD', CMD='SET_MODE', MODE=2)
# Returns: 'TYPE=CMD,CMD=SET_MODE,MODE=2\n'

# Move to specific angles (only in MOVE mode)
build_packet(TYPE='CMD', CMD='JOINTS_TO_ANGLE', current_mode=2,
             JOINT_1_ANGLE=45.5, JOINT_2_ANGLE=67.2, 
             JOINT_3_ANGLE=12.1, JOINT_4_ANGLE=180.0, 
             JOINT_5_ANGLE=90.5, JOINT_6_ANGLE=23.4)

# Emergency stop (only in MOVE mode)
build_packet(TYPE='CMD', CMD='ESTOP', current_mode=2, STOP='ALL')
# Returns: 'TYPE=CMD,CMD=ESTOP,STOP=ALL\n'

# Calibrate joint 3 (only in CALIBRATION mode)
build_packet(TYPE='CMD', CMD='CALIBRATE_JOINT', current_mode=1, JOINT_ID=3)
```

---

### `parse_packet()` Function

**Purpose:** Parse incoming packet strings into dictionaries for processing.

**Signature:**
```python
def parse_packet(packet_str: str) -> Dict[str, Any]
```

**Parameters:**
- `packet_str`: Raw packet (newline stripped)

**Returns:** Dictionary with key-value pairs and TYPE field

**Example:**
```python
parse_packet("TYPE=DATA,CMD=JOINT_ANGLES,ENCODER_1_ANGLE=45.23")
# Returns: {'TYPE': 'DATA', 'CMD': 'JOINT_ANGLES', 'ENCODER_1_ANGLE': '45.23'}
```

---

### `verify_ack()` Function

**Purpose:** Verify that received ACK matches original CMD (except TYPE field).

**Signature:**
```python
def verify_ack(sent_packet_str: str, received_ack_str: str) -> bool
```

**Parameters:**
- `sent_packet_str`: Original CMD packet sent
- `received_ack_str`: Received ACK packet to verify

**Returns:** True if ACK matches CMD

**Behavior:**
- Strips newlines and parses both packets
- Compares all fields except TYPE
- Raises `ProtocolError` on mismatch
- Used by SerialTransceiver (coming next)

**Example:**
```python
sent = "TYPE=CMD,CMD=SET_MODE,MODE=2\n"
ack = "TYPE=ACK,CMD=SET_MODE,MODE=2\n"
verify_ack(sent, ack)  # Returns True
```

---

### Error Handling

All functions raise `ProtocolError` exception with descriptive messages:

```python
try:
    build_packet(TYPE='CMD', CMD='INVALID_CMD')
except ProtocolError as e:
    print(f"Packet error: {e}")
```

---

## Workflow: Building & Sending Commands

```
GUI Button Click (e.g., "SET MODE")
    |
    v
Extract parameters from UI widgets
    |
    v
Call build_packet(TYPE='CMD', CMD='SET_MODE', current_mode=0, MODE=2)
    |
    v [Validates against PROTOCOL_SCHEMAS]
    |
packet_str = 'TYPE=CMD,CMD=SET_MODE,MODE=2\n'
    |
    v
Send packet via serial (SerialTransceiver, coming next)
    |
    v
Wait for ACK
    |
    v
Call verify_ack(sent_packet_str, received_ack_str)
    |
    v [Compares packets]
    |
ACK Valid -> Update UI state, show success
ACK Invalid -> Error popup "Command execution failed: ACK mismatch"
Timeout -> Error popup "Command timed out after 3 attempts"
```

---

## Complete Schema Definition

### TYPE=CMD (UI → Teensy, Requires ACK)

**SET_MODE**
- Allowed modes: All (0, 1, 2, 3)
- Required keys:
  - `MODE`: Integer value from [0, 1, 2, 3] (IDLE, CALIBRATION, MOVE, RESERVED)
- Example: `TYPE=CMD,CMD=SET_MODE,MODE=2`

**JOINTS_TO_ANGLE**
- Allowed modes: Mode 2 (MOVE) only
- Required keys:
  - `JOINT_1_ANGLE`: Float
  - `JOINT_2_ANGLE`: Float
  - `JOINT_3_ANGLE`: Float
  - `JOINT_4_ANGLE`: Float
  - `JOINT_5_ANGLE`: Float
  - `JOINT_6_ANGLE`: Float
- Example: `TYPE=CMD,CMD=JOINTS_TO_ANGLE,JOINT_1_ANGLE=45.5,JOINT_2_ANGLE=67.2,JOINT_3_ANGLE=12.1,JOINT_4_ANGLE=180.0,JOINT_5_ANGLE=90.5,JOINT_6_ANGLE=23.4`

**ESTOP**
- Allowed modes: Mode 2 (MOVE) only
- Required keys:
  - `STOP`: Must be "ALL"
- Example: `TYPE=CMD,CMD=ESTOP,STOP=ALL`

**CALIBRATE_JOINT**
- Allowed modes: Mode 1 (CALIBRATION) only
- Required keys:
  - `JOINT_ID`: Integer from [1, 2, 3, 4, 5, 6]
- Example: `TYPE=CMD,CMD=CALIBRATE_JOINT,JOINT_ID=3`

---

### TYPE=DATA (Teensy → UI, Fire-and-Forget, No ACK)

**JOINT_ANGLES**
- Allowed modes: Mode 1 (CALIBRATION) and Mode 2 (MOVE)
- Required keys:
  - `ENCODER_1_ANGLE`: Float (degrees)
  - `ENCODER_2_ANGLE`: Float (degrees)
  - `ENCODER_3_ANGLE`: Float (degrees)
  - `ENCODER_4_ANGLE`: Float (degrees)
  - `ENCODER_5_ANGLE`: Float (degrees)
  - `ENCODER_6_ANGLE`: Float (degrees)
- Frequency: ~50 Hz (continuous stream)
- Example: `TYPE=DATA,CMD=JOINT_ANGLES,ENCODER_1_ANGLE=45.23,ENCODER_2_ANGLE=67.81,ENCODER_3_ANGLE=12.15,ENCODER_4_ANGLE=180.00,ENCODER_5_ANGLE=90.45,ENCODER_6_ANGLE=23.67`

---

### TYPE=ACK (Teensy → UI, Echoes Original CMD Exactly)

**Format:** Echo the original CMD packet verbatim, only changing TYPE.

**Rules:**
- ACK packet must be **byte-for-byte identical** to the original CMD packet except for the TYPE field
- Teensy replaces `TYPE=CMD` with `TYPE=ACK`
- All other fields, order, and values remain unchanged
- Used for **verification only**: UI compares ACK against sent CMD

**Examples:**

Original CMD:
```
TYPE=CMD,CMD=SET_MODE,MODE=2
```
Expected ACK:
```
TYPE=ACK,CMD=SET_MODE,MODE=2
```

Original CMD:
```
TYPE=CMD,CMD=JOINTS_TO_ANGLE,JOINT_1_ANGLE=45.5,JOINT_2_ANGLE=67.2,JOINT_3_ANGLE=12.1,JOINT_4_ANGLE=180.0,JOINT_5_ANGLE=90.5,JOINT_6_ANGLE=23.4
```
Expected ACK:
```
TYPE=ACK,CMD=JOINTS_TO_ANGLE,JOINT_1_ANGLE=45.5,JOINT_2_ANGLE=67.2,JOINT_3_ANGLE=12.1,JOINT_4_ANGLE=180.0,JOINT_5_ANGLE=90.5,JOINT_6_ANGLE=23.4
```

**Mismatch Detection:**
- If received ACK does not match sent CMD (except TYPE field), UI displays error popup: "Command execution failed: ACK packet mismatch"
- UI allows user to retry or abort operation

---

## Command-Response Flow

```
UI                                  Teensy
|                                   |
|--- TYPE=CMD,CMD=SET_MODE,MODE=2 -->|
|    (UI waits, blocks UI thread)   |
|                                   |[parse packet]
|                                   |[validate: syntax, keys, values, mode]
|                                   |[execute command]
|<-- TYPE=ACK,CMD=SET_MODE,MODE=2 ---|
|                                   |
[UI compares ACK == sent CMD]        |
[if match: success, update UI]      |
[if mismatch: error popup, retry]   |
|                                   |
```

---

## Telemetry Flow (Independent Lane)

```
Teensy continuously streams (no ACK required):

[every ~20ms]
Teensy sends: TYPE=DATA,CMD=JOINT_ANGLES,ENCODER_1_ANGLE=45.23,...

UI receives on separate thread, updates display WITHOUT blocking command lane
```

---

## Timeout & Retry

- **Timeout:** UI waits max 2 seconds for ACK
- **Retries:** Up to 3 attempts before error popup
- **Backoff:** Simple retry (no exponential backoff needed)
- If all 3 retries fail: Error popup "Command timed out after 3 attempts"
