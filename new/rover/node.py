"""
Rover Logic.
This ROS2 Node handles:
1. Reading the camera stream.
2. Subscribing to controller inputs.
3. Calculating arm physics/movement limits.
4. Publishing telemetry/video back to the dashboard.
"""

import json
import time
import cv2

# ROS2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RoverROS(Node):
    def __init__(self):
        super().__init__('RoverROS')
        self.get_logger().info("Rover Node Started. Initializing...")
        
        # --- Camera Setup ---
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera 0.")
        
        self.bridge = CvBridge()
        self.is_cam_allowed = False 

        # --- Robot State ---
        self.arm_angle_1 = 45.0 
        self.arm_angle_2 = 0.0   # New second link starts straight relative to link 1
        
        self.controller_sub = self.create_subscription(
            String, 'controller_inputs', self.controller_inputs_callback, 10
        )

        self.response_pub = self.create_publisher(String, 'controller_inputs_response', 10)
        self.image_pub = self.create_publisher(Image, '/camera_feed', 10)

        self.timer = self.create_timer(1/15.0, self.timer_tick)
        self.get_logger().info("Ready.")

    def timer_tick(self):
        """Reads camera frame and publishes it if allowed."""
        if self.is_cam_allowed and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(ros_image)

    def controller_inputs_callback(self, msg):
        """
        Parses JSON inputs from the dashboard and updates robot state.
        Limits angles to physically plausible values.
        """
        try:
            data = json.loads(msg.data)
            inputs = data.get("inputs", {})
            self.is_cam_allowed = data.get("camera_enabled", False)

            # --- ARM 1 LOGIC (Shoulder) ---
            if "BTN_NORTH" in inputs:
                self.arm_angle_1 += 5.0
            if "BTN_SOUTH" in inputs:
                self.arm_angle_1 -= 5.0
            # Clamp: 0 (Right) to 90 (Up)
            self.arm_angle_1 = max(0.0, min(90.0, self.arm_angle_1))

            # --- ARM 2 LOGIC (Elbow) ---
            # East (Circle/B) and West (Square/X) control the second link
            if "BTN_WEST" in inputs:
                self.arm_angle_2 += 5.0
            if "BTN_EAST" in inputs:
                self.arm_angle_2 -= 5.0
            # Limit elbow to -90 (folded back) to 90 (bent up)
            self.arm_angle_2 = max(-90.0, min(90.0, self.arm_angle_2))

            # --- SEND RESPONSE ---
            packet_id = data.get("id")
            response_payload = {
                "id": packet_id,
                "status": "ACK",
                "rover_timestamp": time.time(),
                "arm_angle_1": self.arm_angle_1,
                "arm_angle_2": self.arm_angle_2
            }
            
            reply = String()
            reply.data = json.dumps(response_payload)
            self.response_pub.publish(reply)

        except Exception as e:
            self.get_logger().error(f"Error processing inputs: {e}")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()