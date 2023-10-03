import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Text_to_cmd_vel(Node):
    
    def __init__(self):
        super().__init__("text_to_cmd_vel_node")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.cmd_text_sub_ = self.create_subscription(String, "cmd_text", self.callback, 10)
        self.get_logger().info(f"Created text_to_cmd_vel_node")

    def callback(self, msg : String):
        self.get_logger().info(f"Get msg: {msg.data}")
        cmd_vel_msg = Twist()
        command = msg.data

        if(command == "turn_right"):
            cmd_vel_msg.angular.z = -1
        elif(command == "turn_left"):
            cmd_vel_msg.angular.z = 1
        elif(command == "move_backward"):
            cmd_vel_msg.linear.x = -1
        elif(command == "move_forward"):
            cmd_vel_msg.linear.x = 1
        
        self.cmd_vel_pub_.publish(cmd_vel_msg)
    
def main(args=None):
    rclpy.init(args=args)
    ttcv = Text_to_cmd_vel()

    rclpy.spin(ttcv)

    ttcv.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()