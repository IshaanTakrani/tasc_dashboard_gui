import os
import subprocess

# Make sure ROS 2 environment is sourced
os.system('bash -c "source /opt/ros/humble/setup.bash && echo setup complete"')

# Now try importing rclpy
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
    import json
    import time
    print("ROS 2 and rclpy are successfully imported!")
except ImportError as e:
    print("ImportError:", e)



class RoverROS(Node):
    def __init__(self):
        super().__init__('RoverROS')
        self.get_logger().info("Rover Node Started. Initializing Camera...")
        
        # self.cap = cv2.VideoCapture(0)

        # New code: Force V4L2 backend
# 0 is usually the index. If you have multiple, try 2 or 10.
        CAMERA_INDEX = 0
        self.cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

# Optional: Force MJPG to prevent timeout on high-bandwidth YUYV
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera 0. Video feed will not work.")
        
        self.bridge = CvBridge()
        # --- CHANGE: Default to FALSE ---
        self.is_cam_allowed = False 
        
        self.controller_sub = self.create_subscription(
            String,
            'controller_inputs',
            self.controller_inputs_callback,
            10
        )

        self.response_pub = self.create_publisher(String, 'controller_inputs_response', 10)
        self.image_pub = self.create_publisher(Image, '/camera_feed', 10)

        self.timer = self.create_timer(1/15.0, self.timer_tick)
        self.get_logger().info("Camera feed ready (Waiting for Dashboard signal).")

    def timer_tick(self):
        if self.is_cam_allowed and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(ros_image)
            else:
                self.get_logger().warn("Could not read frame from camera.")

    def controller_inputs_callback(self, msg):
        try:
            data = json.loads(msg.data)
            
            # Update permission flag (Default to False if missing)
            self.is_cam_allowed = data.get("camera_enabled", False)

            packet_id = data.get("id")
            response_payload = {
                "id": packet_id,
                "status": "ACK",
                "rover_timestamp": time.time()
            }
            reply = String()
            reply.data = json.dumps(response_payload)
            self.response_pub.publish(reply)

        except Exception as e:
            pass

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

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