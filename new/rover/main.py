#!/usr/bin/env python3
"""
Rover Entry Point.
Initializes ROS2 and spins the Rover Node.
"""

import rclpy
from node import RoverROS

def main(args=None):
    rclpy.init(args=args)
    node = RoverROS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()