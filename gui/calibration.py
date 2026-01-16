"""
Calibration Module

Handles 3-point calibration for each joint:
  1. Reference/neutral position (ref_raw)
  2. Max position (max_raw)  
  3. Min position (min_raw)

After capturing raw values, computes logical limits using the mapping formula.
"""

import logging
import config

logger = logging.getLogger(__name__)


class CalibrationState:
    """Tracks calibration progress across joints."""
    
    STEP_REF = 0   # Capture reference/neutral
    STEP_MAX = 1   # Capture max position
    STEP_MIN = 2   # Capture min position
    
    STEP_NAMES = ["ref_raw", "max_raw", "min_raw"]
    STEP_LABELS = ["Reference", "Max", "Min"]
    
    def __init__(self):
        self.joint = None  # Current joint index (0-5), None = not calibrating
        self.step = 0      # Current step (0-2)
        self.last_btn = 0  # For edge detection
    
    @property
    def is_active(self) -> bool:
        """True if calibration is in progress."""
        return self.joint is not None
    
    def start(self):
        """Start calibration from joint 0."""
        self.joint = 0
        self.step = 0
        self.last_btn = 0
        logger.info(f"Calibration started: Joint {self.joint + 1}, Step {self.step}")
    
    def stop(self):
        """Stop calibration."""
        self.joint = None
        self.step = 0
        self.last_btn = 0
        logger.info("Calibration stopped")
    
    def process_button(self, button_state: int) -> bool:
        """Process button state for edge detection.
        
        Args:
            button_state: Current button state (0 or 1)
        
        Returns:
            True if capture should occur (1→0 edge detected)
        """
        should_capture = (self.last_btn == 1 and button_state == 0)
        self.last_btn = button_state
        return should_capture
    
    def capture(self, raw_angle: float, joint_boxes: list) -> bool:
        """Capture a calibration point.
        
        Args:
            raw_angle: Raw encoder angle for current joint
            joint_boxes: List of JointBox widgets for UI updates
        
        Returns:
            True if all joints complete
        """
        if self.joint is None:
            return False
        
        j = self.joint
        step_name = self.STEP_NAMES[self.step]
        step_label = self.STEP_LABELS[self.step]
        
        # Store raw value in config.JOINTS
        config.JOINTS[j][step_name] = raw_angle
        logger.info(f"Calibration: Joint {j + 1} {step_label} (raw)={raw_angle:.1f}°")
        
        # Advance step
        self.step += 1
        
        if self.step > 2:
            # All 3 captures done for this joint
            self._finalize_joint(j, joint_boxes)
            
            # Move to next joint
            self.step = 0
            self.joint += 1
            
            if self.joint >= 6:
                self.joint = None
                logger.info("All joints calibration complete")
                return True
        
        return False
    
    def _finalize_joint(self, j: int, joint_boxes: list):
        """Update UI after joint calibration (logical limits computed from raw values)."""
        jcfg = config.JOINTS[j]
        ref_raw = jcfg["ref_raw"]
        ref_offset = jcfg["ref_offset"]
        direction = jcfg["direction"]
        
        # Compute logical limits from the newly captured raw values
        # (imported function handles direction inversion)
        from angle_mapping import get_logical_limits
        min_logical, max_logical = get_logical_limits(j)
        
        # Update the UI slider with computed logical limits
        if j < len(joint_boxes):
            joint_boxes[j].update_range(
                min_deg=min_logical,
                max_deg=max_logical,
                start_deg=ref_offset
            )
        
        logger.info(f"Joint {j + 1} calibration complete:")
        logger.info(f"  Raw: ref={ref_raw:.1f}, min={jcfg['min_raw']:.1f}, max={jcfg['max_raw']:.1f}")
        logger.info(f"  Logical (computed): ref_offset={ref_offset:.1f}, min={min_logical:.1f}, max={max_logical:.1f}")
    
    def get_status(self) -> str:
        """Get human-readable status string."""
        if not self.is_active:
            return "Idle"
        return f"Joint {self.joint + 1}, Step {self.step + 1}/3 ({self.STEP_LABELS[self.step]})"
