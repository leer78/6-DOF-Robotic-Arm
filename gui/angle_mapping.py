"""
Angle Mapping Utilities

Converts between raw encoder angles (hardware space) and logical angles (application space).
All IK, path planning, and UI should use logical angles.
All Teensy communication uses raw angles.

Config stores only: ref_raw, ref_offset, direction, min_raw, max_raw
Logical limits (min_deg, max_deg) are computed dynamically.

================================================================================
KEY ASSUMPTIONS & CONSTRAINTS (AS5600 encoder wrap-around handling)
================================================================================

1. AS5600 WRAP-AROUND:
   - The AS5600 encoder outputs 0-360° and wraps (e.g., 359° → 1° is a small move).
   - A joint's range may cross the 0°/360° boundary (e.g., ref=305°, max=9.2°).
   - We use "shortest angular distance" to unwrap raw values relative to ref_raw.

2. MAXIMUM JOINT RANGE ASSUMPTION:
   - No joint ever moves more than 180° in either direction from its reference.
   - This allows unambiguous unwrapping: the shortest path is always correct.
   - Example: if ref=305° and logical_max is at raw=9.2°, the shortest path is
     +64.2° (not -295.8°), so we compute (9.2 - 305) wrapped = +64.2°.

3. DIRECTION CONVENTION:
   - direction = +1: Counter-clockwise rotation increases logical angle.
   - direction = -1: Clockwise rotation increases logical angle (inverted magnet).
   - Direction is a multiplier applied AFTER unwrapping the raw delta.

4. CALIBRATION SEMANTICS:
   - min_raw: Raw encoder value at the LOGICAL MINIMUM position.
   - max_raw: Raw encoder value at the LOGICAL MAXIMUM position.
   - The numeric value of max_raw may be less than min_raw if range crosses 0°/360°.

5. FORMULAS (with unwrapping):
   - delta = unwrap(raw - ref_raw)        # Shortest path, range [-180, +180)
   - logical = delta * direction + ref_offset
   - raw = (logical - ref_offset) / direction + ref_raw, then wrap to [0, 360)
================================================================================
"""

import config


def _unwrap_delta(delta: float) -> float:
    """Unwrap an angular delta to the shortest path in range [-180, +180).
    
    This handles AS5600 wrap-around: if raw jumps from 305° to 9°,
    the naive delta is -296°, but the actual movement is +64°.
    
    Args:
        delta: Raw angular difference (may be outside [-180, 180))
    
    Returns:
        Equivalent delta in range [-180, +180)
    """
    while delta >= 180:
        delta -= 360
    while delta < -180:
        delta += 360
    return delta


def _wrap_360(angle: float) -> float:
    """Wrap an angle to [0, 360) range for raw encoder space."""
    angle = angle % 360
    if angle < 0:
        angle += 360
    return angle


def get_logical_limits(joint_idx: int) -> tuple:
    """Compute logical min/max angles from raw calibration values.
    
    Uses unwrapped deltas to handle AS5600 wrap-around correctly.
    
    Args:
        joint_idx: Joint index (0-5)
    
    Returns:
        (min_deg, max_deg) tuple - always min < max
    """
    j = config.JOINTS[joint_idx]
    ref_raw = j["ref_raw"]
    ref_offset = j["ref_offset"]
    direction = j["direction"]
    
    # Unwrap raw deltas relative to reference
    delta_min = _unwrap_delta(j["min_raw"] - ref_raw)
    delta_max = _unwrap_delta(j["max_raw"] - ref_raw)
    
    # Convert to logical
    limit_from_min = delta_min * direction + ref_offset
    limit_from_max = delta_max * direction + ref_offset
    
    # Return as (min, max) regardless of which raw produced which
    return (min(limit_from_min, limit_from_max), max(limit_from_min, limit_from_max))


def raw_to_logical(raw: float, joint_idx: int) -> float:
    """Convert raw encoder angle to logical angle for display/IK.
    
    Handles AS5600 wrap-around by unwrapping the delta from ref_raw.
    
    Args:
        raw: Raw encoder angle from Teensy (0-360)
        joint_idx: Joint index (0-5)
    
    Returns:
        Logical angle clamped to computed [min_deg, max_deg]
    """
    j = config.JOINTS[joint_idx]
    
    # Unwrap delta relative to reference point
    delta = _unwrap_delta(raw - j["ref_raw"])
    
    # Apply direction and offset
    logical = delta * j["direction"] + j["ref_offset"]
    
    # Clamp to computed logical limits
    min_deg, max_deg = get_logical_limits(joint_idx)
    return max(min_deg, min(max_deg, logical))


def logical_to_raw(logical: float, joint_idx: int) -> float:
    """Convert logical angle to raw encoder angle for sending to Teensy.
    
    Inverts the mapping and wraps result to [0, 360) for AS5600.
    
    Args:
        logical: Logical angle from UI/IK
        joint_idx: Joint index (0-5)
    
    Returns:
        Raw encoder angle for Teensy (0-360)
    """
    j = config.JOINTS[joint_idx]
    
    # Clamp logical to computed limits first
    min_deg, max_deg = get_logical_limits(joint_idx)
    logical = max(min_deg, min(max_deg, logical))
    
    # Invert the mapping: delta = (logical - ref_offset) / direction
    delta = (logical - j["ref_offset"]) / j["direction"]
    
    # raw = ref_raw + delta, wrapped to [0, 360)
    raw = _wrap_360(j["ref_raw"] + delta)
    return raw
