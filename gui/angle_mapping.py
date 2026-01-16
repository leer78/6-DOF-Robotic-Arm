"""
Angle Mapping Utilities

Converts between raw encoder angles (hardware space) and logical angles (application space).
All IK, path planning, and UI should use logical angles.
All Teensy communication uses raw angles.

Config stores only: ref_raw, ref_offset, direction, min_raw, max_raw
Logical limits (min_deg, max_deg) are computed dynamically.
"""

import config


def get_logical_limits(joint_idx: int) -> tuple:
    """Compute logical min/max angles from raw calibration values.
    
    Formula: logical = (raw - ref_raw) * direction + ref_offset
    
    Args:
        joint_idx: Joint index (0-5)
    
    Returns:
        (min_deg, max_deg) tuple - always min < max
    """
    j = config.JOINTS[joint_idx]
    ref_raw = j["ref_raw"]
    ref_offset = j["ref_offset"]
    direction = j["direction"]
    
    # Convert raw limits to logical
    limit_a = (j["min_raw"] - ref_raw) * direction + ref_offset
    limit_b = (j["max_raw"] - ref_raw) * direction + ref_offset
    
    # Ensure min < max (direction may invert)
    return (min(limit_a, limit_b), max(limit_a, limit_b))


def raw_to_logical(raw: float, joint_idx: int) -> float:
    """Convert raw encoder angle to logical angle for display/IK.
    
    Formula: logical = (raw - ref_raw) * direction + ref_offset
    
    Args:
        raw: Raw encoder angle from Teensy (0-360)
        joint_idx: Joint index (0-5)
    
    Returns:
        Logical angle clamped to computed [min_deg, max_deg]
    """
    j = config.JOINTS[joint_idx]
    logical = (raw - j["ref_raw"]) * j["direction"] + j["ref_offset"]
    # Clamp to computed logical limits
    min_deg, max_deg = get_logical_limits(joint_idx)
    return max(min_deg, min(max_deg, logical))


def logical_to_raw(logical: float, joint_idx: int) -> float:
    """Convert logical angle to raw encoder angle for sending to Teensy.
    
    Formula: raw = (logical - ref_offset) / direction + ref_raw
    
    Args:
        logical: Logical angle from UI/IK
        joint_idx: Joint index (0-5)
    
    Returns:
        Raw encoder angle for Teensy
    """
    j = config.JOINTS[joint_idx]
    # Clamp logical to computed limits first
    min_deg, max_deg = get_logical_limits(joint_idx)
    logical = max(min_deg, min(max_deg, logical))
    raw = (logical - j["ref_offset"]) / j["direction"] + j["ref_raw"]
    return raw
