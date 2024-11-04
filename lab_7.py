import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import numpy as np

IMAGE_WIDTH = 1400

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')

        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz control loop

        self.target_pos_x = 0.0 # Variable to store the position of the bounding box currently being tracked

    def detection_callback(self, msg):
        pass # Replace with your code

    def timer_callback(self):
        yaw_command = 0.0 # Replace with your code
        forward_vel_command = 0.0

        cmd = Twist()
        cmd.angular.z = yaw_command
        cmd.linear.x = forward_vel_command
        self.command_publisher.publish(cmd)

def main():
    rclpy.init()
    visual_servoing_node = VisualServoingNode()

    try:
        rclpy.spin(visual_servoing_node)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        zero_cmd = Twist()
        visual_servoing_node.command_publisher.publish(zero_cmd)

        visual_servoing_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
