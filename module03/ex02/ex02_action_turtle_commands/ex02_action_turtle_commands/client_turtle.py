import time # BAD ONE

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Twist

from ex02_turtle_action.action import ExecuteTurtleCommand

class TurtleActionClient(Node):

    def __init__(self):
        super().__init__("turtle_action_client")
        self._action_client = ActionClient(self, ExecuteTurtleCommand, 'ExecuteTurtleCommand')
        self.working = False    

    def send_goal(self, cmd, s = 0, angle = 0):
        goal_msg = ExecuteTurtleCommand.Goal()

        goal_msg.cmd = cmd
        goal_msg.s = s
        goal_msg.angle = angle

        self._action_client.wait_for_server()

        self.goal_future = self._action_client.send_goal_async(goal_msg)
        self.goal_future.add_done_callback(self.goal_callback)
        self.working = True
        return self.goal_future

    def goal_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        
        self.get_logger().info("Goal accepted")

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.result}")
        self.working = False

def main():
    rclpy.init()

    tac = TurtleActionClient()

    future = tac.send_goal("forward", 3)
    rclpy.spin_until_future_complete(tac, future)
    future = tac.send_goal("turn_right", angle=90)
    rclpy.spin_until_future_complete(tac, future)
    future = tac.send_goal("forward", 1)
    rclpy.spin_until_future_complete(tac, future)

    rclpy.spin(tac)

    tac.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()