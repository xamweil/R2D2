from __future__ import annotations

import json
from typing import Any, Dict, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from serial_msg.msg import MotorCommand

from behavior_msgs.action import FollowTrack
from behavior_manager.behavior_manager import BehaviorManager
from behavior_manager.behavior_base import BehaviorBase
from behavior_manager.skills.follow_track import FollowTrackSkill

class BehaviorManagerNode(Node):
    """
    ROS-facing shell for the action layer.

    Responsibilities:
    - own publishers/subscribers
    - own action servers
    - cache latest subscribed inputs
    - run the manager update loop at 10 Hz
    - publish status at low rate
    """

    def __init__(self) -> None:
        super().__init__("behavior_manager")

        self.declare_parameter("tracking_topic", "/tracking/tracks")
        self.declare_parameter("motor_command_topic", "/motor_command")
        self.declare_parameter("status_topic", "/behavior_manager/status")
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("status_rate_hz", 2.0)

        # follow_track declarations
        self.declare_parameter("follow_track_head_motor_id", 1)
        self.declare_parameter("follow_track_max_velocity", 80)
        self.declare_parameter("follow_track_center_deadband", 0.08)
        self.declare_parameter("follow_track_bbox_is_normalized", True)
        self.declare_parameter("follow_track_image_width_px", 640)
        self.declare_parameter("follow_track_invert_head_direction", False)

        tracking_topic = self.get_parameter("tracking_topic").get_parameter_value().string_value
        motor_command_topic = self.get_parameter("motor_command_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        update_rate_hz = self.get_parameter("update_rate_hz").get_parameter_value().double_value
        status_rate_hz = self.get_parameter("status_rate_hz").get_parameter_value().double_value

        follow_track_head_motor_id = (
            self.get_parameter("follow_track_head_motor_id").get_parameter_value().integer_value
        )
        follow_track_max_velocity = (
            self.get_parameter("follow_track_max_velocity").get_parameter_value().integer_value
        )
        follow_track_center_deadband = (
            self.get_parameter("follow_track_center_deadband").get_parameter_value().double_value
        )
        follow_track_bbox_is_normalized = (
            self.get_parameter("follow_track_bbox_is_normalized").get_parameter_value().bool_value
        )
        follow_track_image_width_px = (
            self.get_parameter("follow_track_image_width_px").get_parameter_value().integer_value
        )
        follow_track_invert_head_direction = (
            self.get_parameter("follow_track_invert_head_direction").get_parameter_value().bool_value
        )

        self.get_logger().info("behavior_manager started")
        self.get_logger().info(f"  tracking_topic: {tracking_topic}")
        self.get_logger().info(f"  motor_command_topic: {motor_command_topic}")
        self.get_logger().info(f"  status_topic: {status_topic}")
        self.get_logger().info(f"  update_rate_hz: {update_rate_hz}")
        self.get_logger().info(f"  status_rate_hz: {status_rate_hz}")

        self.get_logger().info(f"  follow_track_head_motor_id: {follow_track_head_motor_id}")
        self.get_logger().info(f"  follow_track_max_velocity: {follow_track_max_velocity}")
        self.get_logger().info(f"  follow_track_center_deadband: {follow_track_center_deadband}")
        self.get_logger().info(f"  follow_track_bbox_is_normalized: {follow_track_bbox_is_normalized}")
        self.get_logger().info(f"  follow_track_image_width_px: {follow_track_image_width_px}")
        self.get_logger().info(f"  follow_track_invert_head_direction: {follow_track_invert_head_direction}")

        # Cached inputs
        self._latest_tracking: Optional[Detection2DArray] = None

        # Action goal bookkeeping
        self._follow_track_goal_handle: Optional[ServerGoalHandle] = None

        # Manager
        self._manager = BehaviorManager()

        # Register follow_tracker skill
        self._manager.register_skill(
            FollowTrackSkill(
                head_motor_id=int(follow_track_head_motor_id),
                max_velocity=int(follow_track_max_velocity),
                center_deadband=float(follow_track_center_deadband),
                bbox_is_normalized=bool(follow_track_bbox_is_normalized),
                image_width_px=int(follow_track_image_width_px),
                invert_head_direction=bool(follow_track_invert_head_direction),
            )
        )

        # ROS I/O
        self._tracking_sub = self.create_subscription(
            Detection2DArray,
            tracking_topic,
            self._tracking_callback,
            10,
        )

        self._motor_command_pub = self.create_publisher(
            MotorCommand,
            motor_command_topic,
            10,
        )

        self._status_pub = self.create_publisher(
            String,
            status_topic,
            10,
        )

        # Action server
        self._follow_track_action_server = ActionServer(
            self,
            FollowTrack,
            "follow_track",
            goal_callback=self._follow_track_goal_callback,
            cancel_callback=self._follow_track_cancel_callback,
            execute_callback=self._follow_track_execute_callback,
        )

        # Timers
        self._update_timer = self.create_timer(
            1.0 / update_rate_hz,
            self._update_loop_callback,
        )

        self._status_timer = self.create_timer(
            1.0 / status_rate_hz,
            self._status_timer_callback,
        )

    # ------------------------------------------------------------------
    # ROS subscriptions
    # ------------------------------------------------------------------

    def _tracking_callback(self, msg: Detection2DArray) -> None:
        self._latest_tracking = msg

    # ------------------------------------------------------------------
    # Action server: FollowTrack
    # ------------------------------------------------------------------

    def _follow_track_goal_callback(self, goal_request: FollowTrack.Goal) -> GoalResponse:
        self.get_logger().info(
            f"Received FollowTrack goal for track_id={goal_request.track_id}"
        )

        if goal_request.track_id < 0:
            self.get_logger().warn("Rejecting FollowTrack goal: track_id must be >= 0")
            return GoalResponse.REJECT

        accepted, reason = self._manager.can_start_skill("follow_track")
        if not accepted:
            self.get_logger().warn(f"Rejecting FollowTrack goal: {reason}")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _follow_track_cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().info("Received FollowTrack cancel request")
        return CancelResponse.ACCEPT

    def _follow_track_execute_callback(self, goal_handle: ServerGoalHandle) -> FollowTrack.Result:
        """
        The action execution stays alive while the skill is active.

        The actual periodic work is done by the 10 Hz timer loop.
        """
        import time

        track_id = goal_handle.request.track_id
        self.get_logger().info(f"Starting FollowTrack action for track_id={track_id}")

        self._follow_track_goal_handle = goal_handle
        self._manager.start_skill("follow_track", track_id=track_id)

        result = FollowTrack.Result()

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._manager.stop_skill("follow_track")
                    goal_handle.canceled()
                    result.success = False
                    result.message = "FollowTrack canceled"
                    self.get_logger().info(result.message)
                    return result

                if not self._manager.has_active_skill("follow_track"):
                    goal_handle.succeed()
                    result.success = True
                    result.message = "FollowTrack completed"
                    return result

                time.sleep(0.1)

        finally:
            if self._manager.has_active_skill("follow_track"):
                self._manager.stop_skill("follow_track")

            if self._follow_track_goal_handle == goal_handle:
                self._follow_track_goal_handle = None


    # ------------------------------------------------------------------
    # Timers
    # ------------------------------------------------------------------

    def _update_loop_callback(self) -> None:
        """
        10 Hz manager update loop.

        - gather cached inputs
        - run manager
        - publish outputs
        - publish action feedback if relevant
        """
        inputs: Dict[str, Any] = {
            "tracking": self._latest_tracking,
        }

        outputs = self._manager.update(inputs)

        for output in outputs:
            motor_command = output.get("motor_command")
            if motor_command is not None:
                self._motor_command_pub.publish(motor_command)

            if output.get("skill_name") == "follow_track":
                self._publish_follow_track_feedback(output)

    def _status_timer_callback(self) -> None:
        """
        Low-rate status publisher (default 2 Hz).
        """
        status_msg = String()
        payload = {
            "active_skills": self._manager.active_skill_names(),
            "registered_skills": self._manager.registered_skill_names(),
            "active_resources": sorted(self._manager.active_resources()),
        }
        status_msg.data = json.dumps(payload)
        self._status_pub.publish(status_msg)

    # ------------------------------------------------------------------
    # Feedback helpers
    # ------------------------------------------------------------------

    def _publish_follow_track_feedback(self, output: Dict[str, Any]) -> None:
        if self._follow_track_goal_handle is None:
            return

        if not self._follow_track_goal_handle.is_active:
            return

        feedback_dict = output.get("feedback", {})

        feedback = FollowTrack.Feedback()
        feedback.target_visible = bool(feedback_dict.get("target_visible", False))
        feedback.current_track_id = int(feedback_dict.get("current_track_id", -1))
        feedback.error_x = float(feedback_dict.get("error_x", 0.0))
        feedback.error_y = float(feedback_dict.get("error_y", 0.0))

        self._follow_track_goal_handle.publish_feedback(feedback)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self) -> bool:
        self._manager.stop_all_skills()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = BehaviorManagerNode()

    executor = rclpy.executors.MultiThreadedExecutor()

    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()