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

    def detection_callback(self, msg):
        pass

    def timer_callback(self):
        pass

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
