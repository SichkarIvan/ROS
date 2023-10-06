import time
import sys
import math as m

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class MoveToGoal(Node):

    def __init__(self):
        super().__init__("move_to_goal_node")

        self._cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self._pose_sub    = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.pose_data    = Pose()
        
    def pose_callback(self, msg):
        self.pose_data = msg

    def __call__(self, x, y, theta):
        self.target_x = x
        self.target_y = y
        self.target_theta = theta
        self._timer = self.create_timer(1, self._move_to_goal)
        
    def _get_turtle_pos(self):
        return self.pose_data
    
    def _send_turtle_msg(self, x_speed : float = 0., angle_speed : float = 0.):
        msg = Twist()
        msg.linear.x = x_speed
        msg.angular.z = angle_speed

        self._cmd_vel_pub.publish(msg)

    def _move_to_goal(self):
        x, y, theta = self.target_x, self.target_y, self.target_theta
        start_position = self._get_turtle_pos()

        x_d = x - start_position.x
        y_d = y - start_position.y

        dst_target = m.sqrt(x_d**2 + y_d**2)
        ang_target = m.atan2(y_d, x_d) - start_position.theta

        self.get_logger().info(f"Pos {dst_target} {ang_target}")

        if(abs(dst_target) > 0.1):
            self._send_turtle_msg(dst_target * 0.6, ang_target)
        else:
            self._timer.destroy()
            final_pos = self._get_turtle_pos()
            ang_target = theta - final_pos.theta
            self._send_turtle_msg(angle_speed=ang_target)
            time.sleep(1)
            final_pos = self._get_turtle_pos()
            self.get_logger().info(f"End position: {final_pos.x} {final_pos.y} {final_pos.theta}")

def main():
    rclpy.init()

    mvg = MoveToGoal()
    mvg(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))

    rclpy.spin(mvg)

    mvg.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()