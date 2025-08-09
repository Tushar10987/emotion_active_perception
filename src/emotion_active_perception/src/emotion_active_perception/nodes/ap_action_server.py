from __future__ import annotations

import json
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import String

from emotion_active_perception.action import MoveAndCapture
from ..utils.json_logger import get_json_logger


class ActivePerceptionActionServer(Node):
    def __init__(self) -> None:
        super().__init__("ap_action_server")
        self.logger = get_json_logger("ap_action_server")
        self.emotion_pub = self.create_publisher(String, "/emotion_state", 10)
        self._action_server = ActionServer(
            self,
            MoveAndCapture,
            "move_and_capture",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def goal_callback(self, goal_request: MoveAndCapture.Goal) -> GoalResponse:
        self.logger.info(json.dumps({"event": "goal_received", "request_id": goal_request.request_id}))
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:  # type: ignore[override]
        self.logger.info(json.dumps({"event": "goal_cancel"}))
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle) -> MoveAndCapture.Result:  # type: ignore[override]
        goal: MoveAndCapture.Goal = goal_handle.request
        start = time.time()
        feedback = MoveAndCapture.Feedback()
        info_gain = 0.0
        current_uncertainty = 0.4
        for step in range(3):
            time.sleep(0.3)
            info_gain += 0.05
            current_uncertainty = max(0.0, current_uncertainty - 0.05)
            feedback.info_gain_estimate = float(info_gain)
            feedback.current_uncertainty = float(current_uncertainty)
            feedback.step_count = float(step + 1)
            goal_handle.publish_feedback(feedback)
            self.logger.info(json.dumps({"event": "feedback", "step": step + 1, "ig": info_gain, "u": current_uncertainty}))

        self.emotion_pub.publish(String(data="neutral"))
        result = MoveAndCapture.Result()
        result.success = True
        result.saved_frame_path = ""
        result.final_uncertainty = float(current_uncertainty)
        result.total_time_s = float(time.time() - start)
        goal_handle.succeed()
        self.logger.info(json.dumps({"event": "result", "final_uncertainty": result.final_uncertainty}))
        return result

    


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ActivePerceptionActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

