"""
Test Script for Angle Mapping Functions

Demonstrates the full data flow:
  1. Teensy sends raw encoder angles → GUI displays logical angles
  2. User/IK sets logical angles → GUI sends raw angles to Teensy

Run this from the gui/ directory:
  python test_angle_mapping.py
"""

import config
from angle_mapping import (
    raw_to_logical,
    logical_to_raw,
    get_logical_limits,
    _unwrap_delta,
    _wrap_360
)


def print_separator(title: str):
    print(f"\n{'='*70}")
    print(f" {title}")
    print('='*70)


def print_joint_config(joint_idx: int):
    """Print the configuration for a joint."""
    j = config.JOINTS[joint_idx]
    min_deg, max_deg = get_logical_limits(joint_idx)
    
    print(f"\n  Config: ref_raw={j['ref_raw']}, ref_offset={j['ref_offset']}, direction={j['direction']}")
    print(f"          min_raw={j['min_raw']}, max_raw={j['max_raw']}")
    print(f"  Computed limits: logical min={min_deg:.2f}°, logical max={max_deg:.2f}°")


def test_unwrap_delta():
    """Test the unwrap function that handles 360° wrap-around."""
    print_separator("TEST: _unwrap_delta() - Shortest Angular Path")
    
    test_cases = [
        (50, 50, "No wrap needed"),
        (-50, -50, "Negative, no wrap"),
        (200, -160, "Wrap: 200° → -160° (shorter path is -160°)"),
        (-200, 160, "Wrap: -200° → +160° (shorter path is +160°)"),
        (9.2 - 305, 64.2, "Joint 2 max: 9.2 - 305 = -295.8 → +64.2"),
        (256 - 305, -49, "Joint 2 min: 256 - 305 = -49 (no wrap needed)"),
    ]
    
    print(f"\n  {'Input Delta':<20} {'Expected':<12} {'Actual':<12} {'Pass?'}")
    print(f"  {'-'*20} {'-'*12} {'-'*12} {'-'*6}")
    
    for delta_in, expected, description in test_cases:
        actual = _unwrap_delta(delta_in)
        passed = abs(actual - expected) < 0.01
        status = "✓" if passed else "✗"
        print(f"  {delta_in:<20.2f} {expected:<12.2f} {actual:<12.2f} {status}")
        print(f"    └─ {description}")


def test_joint_2_flow():
    """Test Joint 2 using the current calibration values from config.py."""
    print_separator("TEST: Joint 2 (Shoulder) - Full Flow")
    
    joint_idx = 1
    joint_cfg = config.JOINTS[joint_idx]
    min_deg, max_deg = get_logical_limits(joint_idx)
    print_joint_config(joint_idx)
    
    # Test cases: raw angles the Teensy might send
    raw_test_values = [
        (joint_cfg["ref_raw"], "At reference position"),
        (joint_cfg["max_raw"], "At the captured MAX calibration point"),
        (joint_cfg["min_raw"], "At the captured MIN calibration point"),
        (_wrap_360(joint_cfg["ref_raw"] + 30.0), "30° above reference in raw space"),
        (_wrap_360(joint_cfg["ref_raw"] - 30.0), "30° below reference in raw space"),
    ]
    
    print(f"\n  FLOW 1: Teensy → GUI (raw_to_logical)")
    print(f"  {'Raw (Teensy)':<15} {'Logical (GUI)':<15} {'Description'}")
    print(f"  {'-'*15} {'-'*15} {'-'*30}")
    
    for raw, desc in raw_test_values:
        logical = raw_to_logical(raw, joint_idx)
        print(f"  {raw:<15.2f} {logical:<15.2f} {desc}")
    
    # Test reverse: logical angles for IK
    logical_test_values = [
        (joint_cfg["ref_offset"], f"Reference position (should give raw={joint_cfg['ref_raw']:.1f}°)"),
        (max_deg, "Computed logical maximum"),
        (min_deg, "Computed logical minimum"),
        ((joint_cfg["ref_offset"] + max_deg) / 2.0, "Midpoint in upper range"),
        ((joint_cfg["ref_offset"] + min_deg) / 2.0, "Midpoint in lower range"),
    ]
    
    print(f"\n  FLOW 2: GUI/IK → Teensy (logical_to_raw)")
    print(f"  {'Logical (IK)':<15} {'Raw (Teensy)':<15} {'Description'}")
    print(f"  {'-'*15} {'-'*15} {'-'*30}")
    
    for logical, desc in logical_test_values:
        raw = logical_to_raw(logical, joint_idx)
        print(f"  {logical:<15.2f} {raw:<15.2f} {desc}")


def test_round_trip():
    """Test that raw → logical → raw gives the same value (within joint limits)."""
    print_separator("TEST: Round-Trip Conversion (raw → logical → raw)")
    
    print(f"\n  Testing all 6 joints with various raw values...")
    print(f"  {'Joint':<10} {'Raw In':<12} {'Logical':<12} {'Raw Out':<12} {'Error':<10} {'Pass?'}")
    print(f"  {'-'*10} {'-'*12} {'-'*12} {'-'*12} {'-'*10} {'-'*6}")
    
    all_passed = True
    
    for joint_idx in range(6):
        j = config.JOINTS[joint_idx]
        min_deg, max_deg = get_logical_limits(joint_idx)
        
        # Test a few raw values within the joint's range
        # We'll test: min_raw, ref_raw, max_raw, and a midpoint
        test_raws = [j["min_raw"], j["ref_raw"], j["max_raw"]]
        
        for raw_in in test_raws:
            logical = raw_to_logical(raw_in, joint_idx)
            raw_out = logical_to_raw(logical, joint_idx)
            
            # For wrap-around comparison, check if they're equivalent mod 360
            error = abs(_unwrap_delta(raw_out - raw_in))
            passed = error < 0.1
            status = "✓" if passed else "✗"
            
            if not passed:
                all_passed = False
            
            print(f"  J{joint_idx+1:<8} {raw_in:<12.2f} {logical:<12.2f} {raw_out:<12.2f} {error:<10.4f} {status}")
    
    print(f"\n  Overall: {'ALL TESTS PASSED ✓' if all_passed else 'SOME TESTS FAILED ✗'}")
    return all_passed


def test_ik_scenario():
    """Simulate an IK scenario: desired end-effector position → joint angles → Teensy."""
    print_separator("TEST: IK Scenario Simulation")
    
    print("""
  Scenario: Inverse Kinematics computes these logical joint angles:
    - Joint 1: 45° (base rotation)
    - Joint 2: 120° (shoulder - within 41° to 154.2° range)
    - Joint 3: 30° (elbow)
    - Joint 4: 90° (wrist pitch)
    - Joint 5: 45° (wrist roll)
    - Joint 6: 0° (gripper rotation)
    
  These logical angles need to be converted to raw encoder values for Teensy.
  Then, Teensy will move motors until encoders read these raw values.
  Finally, Teensy reports back raw values, which GUI converts to logical for display.
    """)
    
    ik_logical_angles = [45.0, 120.0, 30.0, 90.0, 45.0, 0.0]
    
    print(f"  {'Joint':<12} {'IK Logical':<12} {'→ Raw (Teensy)':<15} {'→ Logical (verify)':<18} {'Match?'}")
    print(f"  {'-'*12} {'-'*12} {'-'*15} {'-'*18} {'-'*8}")
    
    for joint_idx, ik_angle in enumerate(ik_logical_angles):
        # Clamp to joint limits first
        min_deg, max_deg = get_logical_limits(joint_idx)
        clamped = max(min_deg, min(max_deg, ik_angle))
        
        # Convert to raw for Teensy
        raw = logical_to_raw(clamped, joint_idx)
        
        # Teensy reports back this raw value, convert to logical for display
        logical_back = raw_to_logical(raw, joint_idx)
        
        match = abs(logical_back - clamped) < 0.1
        status = "✓" if match else "✗"
        
        clamped_note = f" (clamped from {ik_angle})" if abs(clamped - ik_angle) > 0.01 else ""
        print(f"  J{joint_idx+1:<10} {clamped:<12.2f} {raw:<15.2f} {logical_back:<18.2f} {status}{clamped_note}")


def test_edge_cases():
    """Test edge cases and boundary conditions."""
    print_separator("TEST: Edge Cases")
    
    print("\n  Testing Joint 2 around the calibrated reference and limits:")
    joint_idx = 1
    joint_cfg = config.JOINTS[joint_idx]
    
    edge_raws = [
        _wrap_360(joint_cfg["ref_raw"] - 2.0),
        _wrap_360(joint_cfg["ref_raw"] - 1.0),
        joint_cfg["ref_raw"],
        _wrap_360(joint_cfg["ref_raw"] + 1.0),
        _wrap_360(joint_cfg["ref_raw"] + 2.0),
        joint_cfg["min_raw"],
        joint_cfg["max_raw"],
    ]
    
    print(f"  {'Raw':<10} {'Logical':<12} {'Note'}")
    print(f"  {'-'*10} {'-'*12} {'-'*30}")
    
    for raw in edge_raws:
        logical = raw_to_logical(raw, joint_idx)
        note = ""
        if abs(raw - joint_cfg["max_raw"]) < 0.01:
            note = "← max_raw position"
        elif abs(raw - joint_cfg["ref_raw"]) < 0.01:
            note = "← ref_raw position"
        elif abs(raw - joint_cfg["min_raw"]) < 0.01:
            note = "← min_raw position"
        print(f"  {raw:<10.2f} {logical:<12.2f} {note}")


def main():
    print("\n" + "="*70)
    print(" ANGLE MAPPING TEST SUITE")
    print(" Verifying raw ↔ logical conversions for AS5600 encoders")
    print("="*70)
    
    # Show all joint configurations
    print_separator("JOINT CONFIGURATIONS (from config.py)")
    for i in range(6):
        j = config.JOINTS[i]
        min_deg, max_deg = get_logical_limits(i)
        print(f"\n  Joint {i+1}: {j['label']}")
        print(f"    Raw:     ref={j['ref_raw']:.1f}°, min={j['min_raw']:.1f}°, max={j['max_raw']:.1f}°")
        print(f"    Logical: ref_offset={j['ref_offset']:.1f}°, min={min_deg:.1f}°, max={max_deg:.1f}°, dir={j['direction']}")
    
    # Run all tests
    test_unwrap_delta()
    test_joint_2_flow()
    test_round_trip()
    test_ik_scenario()
    test_edge_cases()
    
    print_separator("TEST COMPLETE")
    print("\n  Review the output above to verify conversions are correct.")
    print("  Key things to check:")
    j2 = config.JOINTS[1]
    j2_min_deg, j2_max_deg = get_logical_limits(1)
    print(f"    1. Joint 2 logical max should clamp at ≈{j2_max_deg:.1f}°")
    print(f"    2. Joint 2 logical min should clamp at ≈{j2_min_deg:.1f}°")
    print("    3. Round-trip conversions should have near-zero error")
    print("    4. IK angles should survive the round-trip unchanged\n")


if __name__ == "__main__":
    main()
