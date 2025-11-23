#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class RoverROS(Node):
    def __init__(self):
        super().__init__('RoverROS')

        # Subscriber
        self.controller_sub = self.create_subscription(
            String,
            'controller_inputs',
            self.controller_inputs_callback,
            10
        )

        # Publisher
        self.response_pub = self.create_publisher(String, 'controller_inputs_response', 10)

    def controller_inputs_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

        # time.sleep(0.5)
        # Reply back
        reply = String()
        reply.data = f'{msg.data}'  # Echo the original message
        self.response_pub.publish(reply)
        self.get_logger().info(f'Sent reply.')

def main(args=None):
    rclpy.init(args=args)
    node = RoverROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()