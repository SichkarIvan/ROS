import time # BAD ONE
from math import pi as m_pi

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from geometry_msgs.msg import Twist

from ex02_turtle_action.action import ExecuteTurtleCommand

class TurtleAction(Node):

    def __init__(self):
        super().__init__("turtle_action_server")
        self.odometer = 0

        self._cmd_vel_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self._action_server = ActionServer(
            self, 
            ExecuteTurtleCommand,
            'ExecuteTurtleCommand',
            self.execute_callback
        )
    
    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Accepted goal cmd: {goal_handle.request.cmd}")

        cmd_vel_msg = Twist()

        feedback_msg = ExecuteTurtleCommand.Feedback()
        feedback_msg.odom = self.odometer

        cmd = goal_handle.request.cmd

        if(cmd == "forward"):
            cmd_vel_msg.linear.x = 1.
            for i in range(goal_handle.request.s):
                self._cmd_vel_pub.publish(cmd_vel_msg)
                self.odometer += 1
                feedback_msg.odom = self.odometer
                self._cmd_vel_pub.publish(cmd_vel_msg)
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.5)
        elif(cmd == "turn_right"):
            cmd_vel_msg.angular.z = -m_pi/2
            self._cmd_vel_pub.publish(cmd_vel_msg)
            goal_handle.publish_feedback(feedback_msg)
        elif(cmd == "turn_left"):
            cmd_vel_msg.angular.z = m_pi/2
            self._cmd_vel_pub.publish(cmd_vel_msg)
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = ExecuteTurtleCommand.Result()
        result.result = True
        return result

def main():
    rclpy.init()

    ta = TurtleAction()

    rclpy.spin(ta)

    ta.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()