# rover/main.py
#!/usr/bin/env python3
import os
import sys

# Source ROS2 environment
os.system('bash -c "source /opt/ros/humble/setup.bash && echo ROS2 Environment Sourced"')

try:
    import rclpy
    from node import RoverROS
except ImportError:
    print("Failed to import ROS2. Ensure source /opt/ros/humble/setup.bash is run.")
    sys.exit(1)

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