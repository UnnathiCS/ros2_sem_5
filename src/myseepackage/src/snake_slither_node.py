#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class SnakeSlither(Node):
    def __init__(self):
        super().__init__('snake_slither')

        # List of joint command topics
        self.joint_names = [
            'snake_body_1_joint',
            'snake_body_2_joint',
            'snake_body_3_joint',
            'snake_body_4_joint',
            'snake_body_5_joint',
            'snake_body_6_joint',
            'snake_body_7_joint'
        ]

        # Publishers for each joint
        self.joint_pubs = []
        for joint in self.joint_names:
            pub = self.create_publisher(Float64, f'/{joint}/command', 10)
            self.joint_pubs.append(pub)

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.start_time = time.time()

    def timer_callback(self):
        t = time.time() - self.start_time
        for i, pub in enumerate(self.joint_pubs):
            msg = Float64()
            # Sine wave oscillation for slithering
            msg.data = 0.5 * math.sin(2.0 * t + i * 0.5)
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SnakeSlither()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
