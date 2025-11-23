# dashboard/ros_interface.py
import time
import json
from collections import deque
import cv2
import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6.QtGui import QImage

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Image as RosImage
    from cv_bridge import CvBridge
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

class RosWorker(QThread):
    stats_signal = pyqtSignal(float, float, int, float, bool)
    video_signal = pyqtSignal(QImage)
    telemetry_signal = pyqtSignal(float) # New signal for Arm Angle
    
    def __init__(self):
        super().__init__()
        self.active = True
        self.node = None 

    def send_controller_state(self, state_dict):
        if self.node: self.node.update_input_cache(state_dict)

    def run(self):
        if not ROS_AVAILABLE: return
        rclpy.init(args=None)
        self.node = DashboardROSNode(self)
        while self.active and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.node.destroy_node()
        rclpy.shutdown()

    def stop(self):
        self.active = False; self.wait()

class DashboardROSNode(Node):
    def __init__(self, worker):
        super().__init__('DashboardROS')
        self.worker = worker
        self.pub = self.create_publisher(String, 'controller_inputs', 10)
        self.sub = self.create_subscription(String, 'controller_inputs_response', self.callback, 10)
        self.bridge = CvBridge()
        self.sub_cam = self.create_subscription(RosImage, '/camera_feed', self.image_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_tick)
        
        self.current_inputs = {} 
        self.msg_id = 0; self.sent_t = {}
        self.lats = deque(maxlen=20)
        self.rx = 0; self.tx = 0; self.last_rx_time = 0; self.current_rate = 0.0
        self.camera_enabled = False 

    def update_input_cache(self, inputs_dict):
        self.current_inputs = inputs_dict

    def image_callback(self, msg):
        if not self.camera_enabled: return 
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_img.shape
            bytes_per_line = ch * w
            qt_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
            self.worker.video_signal.emit(qt_img)
        except Exception: pass

    def timer_tick(self):
        is_connected = (time.time() - self.last_rx_time) < 1.0
        if not is_connected: self.current_rate = 0.0; lat = 0.0; stbl = 0.0
        else:
            lat = self.lats[-1] if self.lats else 0
            stbl = (self.rx/self.tx)*100 if self.tx > 0 else 0
        loss = self.tx - self.rx
        self.worker.stats_signal.emit(float(lat), float(stbl), int(loss), float(self.current_rate), is_connected)

        packet = {"id": self.msg_id, "timestamp": time.time(), "inputs": self.current_inputs, "camera_enabled": self.camera_enabled}
        msg = String(); msg.data = json.dumps(packet)
        self.pub.publish(msg)
        self.sent_t[self.msg_id] = time.time(); self.tx += 1; self.msg_id += 1

    def callback(self, msg):
        try:
            now = time.time(); data = json.loads(msg.data); mid = data.get("id", -1)
            
            # Extract Arm telemetry
            arm_angle = data.get("arm_angle", 90.0)
            self.worker.telemetry_signal.emit(float(arm_angle))

            if mid in self.sent_t:
                lat = (now - self.sent_t[mid]) * 1000
                del self.sent_t[mid]
                if self.last_rx_time > 0:
                    delta = now - self.last_rx_time
                    if delta > 0: self.current_rate = (self.current_rate * 0.7) + ((1.0/delta) * 0.3)
                self.last_rx_time = now; self.lats.append(lat); self.rx += 1
        except: pass