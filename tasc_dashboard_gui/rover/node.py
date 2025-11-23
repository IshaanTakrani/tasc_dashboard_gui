# rover/node.py
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
    import json
    import time
except ImportError:
    print("ROS2 Imports failed. Make sure environment is sourced.")

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
        self.arm_angle = 45.0 
        
        self.controller_sub = self.create_subscription(String, 'controller_inputs', self.controller_inputs_callback, 10)
        self.response_pub = self.create_publisher(String, 'controller_inputs_response', 10)
        self.image_pub = self.create_publisher(Image, '/camera_feed', 10)
        self.timer = self.create_timer(1/15.0, self.timer_tick)
        self.get_logger().info("Ready.")

    def timer_tick(self):
        if self.is_cam_allowed and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(ros_image)

    def controller_inputs_callback(self, msg):
        try:
            data = json.loads(msg.data)
            inputs = data.get("inputs", {})
            self.is_cam_allowed = data.get("camera_enabled", False)

            # --- ARM LOGIC ---
            if "BTN_NORTH" in inputs: self.arm_angle += 5.0
            if "BTN_SOUTH" in inputs: self.arm_angle -= 5.0
            self.arm_angle = max(0.0, min(90.0, self.arm_angle))

            # --- SEND RESPONSE ---
            response_payload = {
                "id": data.get("id"),
                "status": "ACK",
                "rover_timestamp": time.time(),
                "arm_angle": self.arm_angle
            }
            reply = String(); reply.data = json.dumps(response_payload)
            self.response_pub.publish(reply)
        except Exception as e:
            self.get_logger().error(f"Error processing inputs: {e}")

    def destroy_node(self):
        if self.cap.isOpened(): self.cap.release()
        super().destroy_node()