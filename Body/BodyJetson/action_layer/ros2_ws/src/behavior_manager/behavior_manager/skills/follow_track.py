from __future__ import annotations

from typing import Any, Dict, Optional

from serial_msg.msg import MotorCommand
from vision_msgs.msg import Detection2DArray

from behavior_manager.behavior_base import BehaviorBase


class FollowTrackSkill(BehaviorBase):
    """
    Very simple follow behavior.

    Behavior:
    - ignore y completely
    - align target box center with frame center in x direction
    - output head motor velocity commands only
    - stop inside deadband
    """

    def __init__(
        self,
        head_motor_id: int = 1,
        max_velocity: int = 80,
        center_deadband: float = 0.08,
        bbox_is_normalized: bool = True,
        image_width_px: int = 640,
        invert_head_direction: bool = True,
    ) -> None:
        self._head_motor_id = head_motor_id
        self._max_velocity = max_velocity
        self._center_deadband = center_deadband
        self._bbox_is_normalized = bbox_is_normalized
        self._image_width_px = image_width_px
        self._invert_head_direction = invert_head_direction

        self._track_id: int = -1
        self._running: bool = False

    @property
    def name(self) -> str:
        return "follow_track"

    def required_resources(self) -> set[str]:
        return {"head_motion"}

    def start(self, **kwargs: Any) -> None:
        self._track_id = int(kwargs.get("track_id", -1))
        self._running = True

    def stop(self) -> None:
        self._running = False

    def update(self, inputs: Dict[str, Any]) -> Dict[str, Any]:
        tracking_msg: Optional[Detection2DArray] = inputs.get("tracking")

        if not self._running:
            return {
                "motor_command": self._make_stop_command(),
                "feedback": {
                    "target_visible": False,
                    "current_track_id": self._track_id,
                    "error_x": 0.0,
                    "error_y": 0.0,
                },
                "done": True,
                "success": False,
                "message": "follow_track not running",
            }

        detection = self._find_target_detection(tracking_msg, self._track_id)

        if detection is None:
            return {
                "motor_command": self._make_stop_command(),
                "feedback": {
                    "target_visible": False,
                    "current_track_id": self._track_id,
                    "error_x": 0.0,
                    "error_y": 0.0,
                },
                "done": False,
                "success": False,
                "message": "target not visible",
            }

        center_x = self._extract_center_x(detection)

        if center_x is None:
            return {
                "motor_command": self._make_stop_command(),
                "feedback": {
                    "target_visible": False,
                    "current_track_id": self._track_id,
                    "error_x": 0.0,
                    "error_y": 0.0,
                },
                "done": False,
                "success": False,
                "message": "could not extract bbox center x",
            }

        error_x = self._compute_normalized_error(center_x)

        if abs(error_x) <= self._center_deadband:
            motor_command = self._make_stop_command()
        else:
            motor_command = self._make_velocity_command(error_x)

        return {
            "motor_command": motor_command,
            "feedback": {
                "target_visible": True,
                "current_track_id": self._track_id,
                "error_x": float(error_x),
                "error_y": 0.0,
            },
            "done": False,
            "success": False,
            "message": "follow_track running",
        }

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _find_target_detection(
        self,
        tracking_msg: Optional[Detection2DArray],
        track_id: int,
    ):
        if tracking_msg is None:
            return None

        for detection in tracking_msg.detections:
            detection_id = getattr(detection, "id", "")
            if str(detection_id) == str(track_id):
                return detection

        return None

    def _extract_center_x(self, detection) -> Optional[float]:
        """
        Support both bbox.center.x and bbox.center.position.x styles,
        depending on the installed vision_msgs version / previous code.
        """
        bbox = getattr(detection, "bbox", None)
        if bbox is None:
            return None

        center = getattr(bbox, "center", None)
        if center is None:
            return None

        # Some message variants / user code paths
        if hasattr(center, "x"):
            return float(center.x)

        if hasattr(center, "position") and hasattr(center.position, "x"):
            return float(center.position.x)

        return None

    def _compute_normalized_error(self, center_x: float) -> float:
        """
        Returns signed x-error normalized roughly to [-1, 1].

        Negative: target left of image center
        Positive: target right of image center
        """
        if self._bbox_is_normalized:
            # Expect center_x in [0, 1]
            image_center_x = 0.5
            half_width = 0.5
        else:
            # Expect center_x in pixel coordinates
            image_center_x = self._image_width_px / 2.0
            half_width = self._image_width_px / 2.0

        return (center_x - image_center_x) / half_width

    def _make_velocity_command(self, error_x: float) -> MotorCommand:
        """
        Linear proportional mapping:
            outside deadband -> velocity in [1, max_velocity]
        """
        magnitude = abs(error_x)

        # Normalize outside the deadband into [0, 1]
        scaled = (magnitude - self._center_deadband) / max(1e-6, (1.0 - self._center_deadband))
        scaled = max(0.0, min(1.0, scaled))

        velocity = int(round(scaled * self._max_velocity))
        velocity = max(1, min(self._max_velocity, velocity))

        # Base direction: error_x > 0 means target is right of center
        direction = error_x > 0.0

        if self._invert_head_direction:
            direction = not direction

        msg = MotorCommand()
        msg.ids = [self._head_motor_id]
        msg.enable = [True]
        msg.direction = [direction]
        msg.angle_set = [False]
        msg.velocity_set = [True]
        msg.angle = [0.0]
        msg.velocity = [velocity]
        return msg

    def _make_stop_command(self) -> MotorCommand:
        msg = MotorCommand()
        msg.ids = [self._head_motor_id]
        msg.enable = [False]
        msg.direction = [False]
        msg.angle_set = [False]
        msg.velocity_set = [False]
        msg.angle = [0.0]
        msg.velocity = [0]
        return msg