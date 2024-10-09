#!/usr/bin/env python3

# PART 2: Initial Imports
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.msgs.msg import TurtleBotControl, TurtleBotState
import rclpy
from rclpy.node import Node

# PART 3: Define the HeadingController Class
class HeadingController(BaseHeadingController):
    def __init__(self) -> None:
        BaseHeadingController.__init__(self)
        self.kp = 2.0
    
    # PART 4: Proportional Control
    def compute_control_with_goal(self, current: TurtleBotState, desired: TurtleBotState) -> TurtleBotControl:
        heading_error = wrap_angle(desired.theta - current.theta) # [-pi,pi]
        omega_correction = self.kp * heading_error
        new_state = TurtleBotControl()
        new_state.omega = omega_correction
        return new_state
    
# PART 5: Node Execution
if __name__ == "__main__":
    rclpy.init()
    node = HeadingController()
    rclpy.spin(node)
    rclpy.shutdown()