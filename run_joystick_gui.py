

# import sys
# from PyQt6.QtCore import *
# from PyQt6.QtGui import *
# from PyQt6.QtWidgets import *
# from evdev import InputDevice, categorize, ecodes


# DEVICE_PATH = "/dev/input/event6"   # <- change this!


# BUTTON_ALIAS = {
#     "BTN_SOUTH": "BTN_SOUTH",
#     "BTN_A":     "BTN_SOUTH",

#     "BTN_EAST":  "BTN_EAST",
#     "BTN_B":     "BTN_EAST",

#     "BTN_NORTH": "BTN_NORTH",
#     "BTN_X":     "BTN_NORTH",

#     "BTN_WEST":  "BTN_WEST",
#     "BTN_Y":     "BTN_WEST",

#         # NEW:
#     "BTN_TL": "L1",
#     "BTN_TR": "R1",
# }



# # -----------------------------
# # Worker Thread (evdev reader)
# # -----------------------------
# class ControllerThread(QThread):
#     axisChanged = pyqtSignal(str, int)
#     buttonChanged = pyqtSignal(str, int)

#     def run(self):
#         gamepad = InputDevice(DEVICE_PATH)

#         for event in gamepad.read_loop():

#             # Handle buttons
#             if event.type == ecodes.EV_KEY:
#                 key = categorize(event)
#                 keycode = key.keycode
#                 print("KEY EVENT:", key, "VALUE:", keycode)
#                 # Ensure keycode is always a string
#                 if isinstance(keycode, list):
#                     keycode = keycode[0]

#                 self.buttonChanged.emit(keycode, event.value)

#             # Handle axes
            
#             elif event.type == ecodes.EV_ABS:
#                 code = ecodes.ABS[event.code]
#                 print("ABS EVENT:", code, "VALUE:", event.value)    # <--- ADD THIS
#                 self.axisChanged.emit(code, event.value)


# # -# -----------------------------
# # GUI Window
# # -----------------------------
# # -----------------------------
# # GUI Window
# # -----------------------------
# class ControllerGUI(QWidget):
#     def __init__(self):
#         super().__init__()

#         layout = QVBoxLayout(self)

#         # ==========================================================
#         #  TOP ROW: L2 and R2 triggers
#         # ==========================================================
#         trigger_row = QHBoxLayout()

#         self.l2_bar = QProgressBar()
#         self.l2_bar.setRange(0, 255)
#         self.l2_bar.setFormat("L2")
#         trigger_row.addWidget(self.l2_bar)

#         self.r2_bar = QProgressBar()
#         self.r2_bar.setRange(0, 255)
#         self.r2_bar.setFormat("R2")
#         trigger_row.addWidget(self.r2_bar)

#         layout.addLayout(trigger_row)


#         # ==========================================================
#         #  MIDDLE TOP: L1 and R1 labels
#         # ==========================================================
#         shoulder_row = QHBoxLayout()

#         self.l1_label = QLabel("L1")
#         self.l1_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
#         self.l1_label.setStyleSheet("padding: 10px; background:#444; color:white; border-radius:6px;")
#         shoulder_row.addWidget(self.l1_label)

#         self.r1_label = QLabel("R1")
#         self.r1_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
#         self.r1_label.setStyleSheet("padding: 10px; background:#444; color:white; border-radius:6px;")
#         shoulder_row.addWidget(self.r1_label)

#         self.button_labels = {}

#         self.button_labels["L1"] = self.l1_label
#         self.button_labels["R1"] = self.r1_label


#         layout.addLayout(shoulder_row)


#         # ==========================================================
#         #  JOYSTICKS (Left / Right)
#         # ==========================================================
#         joysticks_row = QHBoxLayout()

#         self.left_joystick_widget = JoystickWidget()
#         joysticks_row.addWidget(self.left_joystick_widget)

#         self.right_joystick_widget = JoystickWidget()
#         joysticks_row.addWidget(self.right_joystick_widget)

#         layout.addLayout(joysticks_row)


#         # ==========================================================
#         #  TOUCHPAD rectangle (visual only)
#         # ==========================================================
#         touchpad = QLabel("Touchpad")
#         touchpad.setAlignment(Qt.AlignmentFlag.AlignCenter)
#         touchpad.setMinimumHeight(40)
#         touchpad.setStyleSheet(
#             "background:#2b2b2b; color:white; border:2px solid #666; border-radius:6px;"
#         )
#         layout.addWidget(touchpad)


#         # ==========================================================
#         #  FACE BUTTONS (PS4 diamond layout)
#         # ==========================================================
#         face_button_layout = QGridLayout()

#         self.button_labels = {
#             "BTN_NORTH": QLabel("▲"),  # Triangle (Green)
#             "BTN_SOUTH": QLabel("✖"),  # Cross (Blue)
#             "BTN_WEST":  QLabel("■"),  # Square (Pink)
#             "BTN_EAST":  QLabel("●"),  # Circle (Red)
#         }

#         base_style = "background:#333; color:white; padding:15px; font-size:20px; border-radius:8px;"

#         for lbl in self.button_labels.values():
#             lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
#             lbl.setStyleSheet(base_style)

#         # Diamond layout
#         face_button_layout.addWidget(self.button_labels["BTN_NORTH"], 0, 1)  # top
#         face_button_layout.addWidget(self.button_labels["BTN_WEST"],  1, 0)  # left
#         face_button_layout.addWidget(self.button_labels["BTN_EAST"],  1, 2)  # right
#         face_button_layout.addWidget(self.button_labels["BTN_SOUTH"], 2, 1)  # bottom

#         layout.addLayout(face_button_layout)


#         # ==========================================================
#         #  D-PAD section
#         # ==========================================================
#         dpad_layout = QGridLayout()

#         self.dpad_labels = {
#             "DPAD_UP": QLabel("↑"),
#             "DPAD_DOWN": QLabel("↓"),
#             "DPAD_LEFT": QLabel("←"),
#             "DPAD_RIGHT": QLabel("→"),
#         }

#         dpad_style = "background:#222; color:white; padding:12px; font-size:20px; border-radius:8px;"

#         for lbl in self.dpad_labels.values():
#             lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
#             lbl.setStyleSheet(dpad_style)

#         # Diamond layout
#         dpad_layout.addWidget(self.dpad_labels["DPAD_UP"],    0, 1)
#         dpad_layout.addWidget(self.dpad_labels["DPAD_LEFT"],  1, 0)
#         dpad_layout.addWidget(self.dpad_labels["DPAD_RIGHT"], 1, 2)
#         dpad_layout.addWidget(self.dpad_labels["DPAD_DOWN"],  2, 1)

#         layout.addSpacing(10)
#         layout.addLayout(dpad_layout)



#         # Thread
#         self.thread = ControllerThread()
#         self.thread.axisChanged.connect(self.update_axis)
#         self.thread.buttonChanged.connect(self.update_button)
#         self.thread.start()



#     def update_dpad(self, name, pressed):
#         if name not in self.dpad_labels:
#             return

#         if pressed:
#             self.dpad_labels[name].setStyleSheet(
#                 "background: yellow; color: black; padding: 12px; "
#                 "font-size: 20px; border-radius: 8px;"
#             )
#         else:
#             self.dpad_labels[name].setStyleSheet(
#                 "background: #222; color: white; padding: 12px; "
#                 "font-size: 20px; border-radius: 8px;"
#             )




#     # -----------------------------
#     # Update joystick + triggers
#     # -----------------------------
#     def update_axis(self, axis_name, value):

#         # ---- Joysticks ----
#         norm = (value - 128) / 128.0

#         if axis_name == "ABS_X":
#             self.left_x = norm
#         elif axis_name == "ABS_Y":
#             self.left_y = norm
#         elif axis_name == "ABS_RX":
#             self.right_x = norm
#         elif axis_name == "ABS_RY":
#             self.right_y = norm

#         # ---- Triggers (0–255) ----
#         elif axis_name == "ABS_Z":
#             self.l2 = value
#             self.l2_bar.setValue(value)

#         elif axis_name == "ABS_RZ":
#             self.r2 = value
#             self.r2_bar.setValue(value)


#                     # DPAD (HAT 0)
#         if axis_name == "ABS_HAT0X":
#             if value == -1:
#                 self.update_dpad("DPAD_LEFT", True)
#                 self.update_dpad("DPAD_RIGHT", False)
#             elif value == 1:
#                 self.update_dpad("DPAD_RIGHT", True)
#                 self.update_dpad("DPAD_LEFT", False)
#             else:
#                 self.update_dpad("DPAD_LEFT", False)
#                 self.update_dpad("DPAD_RIGHT", False)

#         elif axis_name == "ABS_HAT0Y":
#             if value == -1:
#                 self.update_dpad("DPAD_UP", True)
#                 self.update_dpad("DPAD_DOWN", False)
#             elif value == 1:
#                 self.update_dpad("DPAD_DOWN", True)
#                 self.update_dpad("DPAD_UP", False)
#             else:
#                 self.update_dpad("DPAD_UP", False)
#                 self.update_dpad("DPAD_DOWN", False)


#         # Update joystick drawings
#         self.left_joystick_widget.update_position(self.left_x, self.left_y)
#         self.right_joystick_widget.update_position(self.right_x, self.right_y)


#     # -----------------------------
#     # Update button highlight
#     # -----------------------------
#     # def update_button(self, button_name, pressed):
#     #     if button_name in BUTTON_ALIAS:
#     #         button_name = BUTTON_ALIAS[button_name]
#     #     else:
#     #         return

#     #     if pressed:
#     #         self.button_labels[button_name].setStyleSheet(
#     #             "background: green; color: black; padding: 10px; border-radius: 5px;"
#     #         )
#     #     else:
#     #         self.button_labels[button_name].setStyleSheet(
#     #             "background: #444; color: white; padding: 10px; border-radius: 5px;"
#     #         )

#     def update_button(self, button_name, pressed):
#     # Convert aliases (including L1/R1)
#         if button_name in BUTTON_ALIAS:
#             button_name = BUTTON_ALIAS[button_name]
#         else:
#             return

#         if button_name not in self.button_labels:
#             return

#         # Styles
#         active_style = (
#             "background: yellow; color: black; padding: 10px; "
#             "border-radius: 6px;"
#         )
#         inactive_style = (
#             "background: #444; color: white; padding: 10px; "
#             "border-radius: 6px;"
#         )

#         # Apply highlight
#         if pressed:
#             self.button_labels[button_name].setStyleSheet(active_style)
#         else:
#             self.button_labels[button_name].setStyleSheet(inactive_style)






# # -----------------------------
# # Joystick Visualization Widget
# # -----------------------------
# class JoystickWidget(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.x = 0
#         self.y = 0
#         self.setMinimumSize(200, 200)

#     def update_position(self, x, y):
#         """x & y normalized: -1 to 1"""
#         self.x = x
#         self.y = y
#         self.update()

#     def paintEvent(self, event):
#         painter = QPainter(self)
#         painter.setRenderHint(QPainter.RenderHint.Antialiasing)

#         w, h = self.width(), self.height()
#         center = QPointF(w / 2, h / 2)
#         radius = min(w, h) * 0.40

#         # Outer circle
#         painter.setPen(QPen(Qt.GlobalColor.white, 3))
#         painter.drawEllipse(center, radius, radius)

#         # Inner dot (joystick position)
#         dot_x = center.x() + self.x * radius
#         dot_y = center.y() + self.y * radius

#         painter.setBrush(QBrush(Qt.GlobalColor.green))
#         painter.drawEllipse(QPointF(dot_x, dot_y), 10, 10)


# # -----------------------------
# # Run App
# # -----------------------------
# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = ControllerGUI()
#     window.show()
#     sys.exit(app.exec())



#!/usr/bin/env python3
import sys
import time
from collections import deque
import json

# PyQt6 Imports
from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *

# Input Imports
from evdev import InputDevice, categorize, ecodes

# ROS2 Imports
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:
    print("WARNING: 'rclpy' not found. ROS features will be disabled.")
    ROS_AVAILABLE = False

# =============================================================================
# CONFIGURATION
# =============================================================================

DEVICE_PATH = "/dev/input/event6"  # <- CHANGE THIS to your device path

# =============================================================================
# THEME COLORS (Dark Matte DS4 Style)
# =============================================================================

COLOR_BG       = "#1A1A1D"
COLOR_SURFACE  = "#2C2C31"
COLOR_TOUCHPAD = "#151518"
COLOR_LIGHT    = "#484850"
COLOR_SHADOW   = "#0A0A0C"
COLOR_TEXT     = "#A0A0A5"
COLOR_TEXT_LIT = "#FFFFFF"
COLOR_ACCENT   = "#5271FF"

# Button Colors
BTN_COLORS = {
    "Triangle": "#2ECC71", # Green
    "Circle":   "#E74C3C", # Red
    "Cross":    "#3498DB", # Blue
    "Square":   "#E056FD", # Pink
    "Default":  "#95A5A6"
}

# =============================================================================
# ROS2 WORKER THREAD
# =============================================================================

class RosWorker(QThread):
    # Signals to update GUI
    stats_signal = pyqtSignal(float, float, int, float) # Latency, Avg Latency, Loss, Stability
    log_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.active = True
        self.latest_controller_state = "No Input"

    def update_controller_state(self, state_str):
        self.latest_controller_state = state_str

    def run(self):
        if not ROS_AVAILABLE:
            return

        rclpy.init(args=None)
        # Create the node defined in your requirements
        self.node = DashboardROSNode(self)
        
        # Spin loop
        while self.active and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
        
        self.node.destroy_node()
        rclpy.shutdown()

    def stop(self):
        self.active = False
        self.wait()

class DashboardROSNode(Node):
    def __init__(self, worker_thread):
        super().__init__('DashboardROS')
        self.worker = worker_thread

        # Publisher
        self.controller_pub = self.create_publisher(String, 'controller_inputs', 10)

        # Subscriber
        self.controller_sub = self.create_subscription(
            String,
            'controller_inputs_response',
            self.controller_inputs_callback,
            10
        )

        # Timer (1.0s interval as per requirements)
        self.timer = self.create_timer(1.0, self.send_controller_data)
        self.message_id = 0

        # Metrics
        self.sent_times = {} 
        self.received_count = 0
        self.sent_count = 0
        self.latencies = deque(maxlen=20)

    def send_controller_data(self):
        # Embed the actual controller state from the GUI into the message
        payload = self.worker.latest_controller_state
        
        msg = String()
        # Formatting exactly as required for parsing, but adding payload
        msg.data = f'Message ID: {self.message_id}\n{payload}'
        
        self.controller_pub.publish(msg)
        # self.get_logger().info(f'Sent: {msg.data}') # Optional: Reduce console spam

        self.sent_times[self.message_id] = time.time()
        self.sent_count += 1
        self.message_id += 1

    def controller_inputs_callback(self, msg):
        # Extract message ID
        try:
            # Robust parsing looking for "Message ID: X"
            lines = msg.data.split('\n')
            id_line = [l for l in lines if "Message ID:" in l][0]
            msg_id_str = id_line.split("Message ID: ")[1]
            msg_id = int(msg_id_str)
        except (IndexError, ValueError):
            return

        if msg_id in self.sent_times:
            # Calculate Stats
            latency = (time.time() - self.sent_times[msg_id]) * 1000
            self.latencies.append(latency)
            self.received_count += 1
            del self.sent_times[msg_id]

            avg_latency = sum(self.latencies) / len(self.latencies)
            lost_packets = self.sent_count - self.received_count
            stability = (self.received_count / self.sent_count) * 100 if self.sent_count else 0

            # Send data to GUI via Signal
            self.worker.stats_signal.emit(latency, avg_latency, lost_packets, stability)


# =============================================================================
# CONTROLLER INPUT THREAD
# =============================================================================

class ControllerThread(QThread):
    axisChanged = pyqtSignal(str, int)
    buttonChanged = pyqtSignal(str, int)
    # Emits raw string representation of inputs for ROS
    stateUpdated = pyqtSignal(str) 

    def run(self):
        try:
            gamepad = InputDevice(DEVICE_PATH)
            print(f"Listening to {gamepad.name}...")
            
            # Keep track of active inputs for the ROS payload
            active_inputs = {} 

            for event in gamepad.read_loop():
                if event.type == ecodes.EV_KEY:
                    key = categorize(event)
                    keycode = key.keycode if not isinstance(key.keycode, list) else key.keycode[0]
                    self.buttonChanged.emit(keycode, event.value)
                    
                    # Update state dict
                    if event.value == 1:
                        active_inputs[keycode] = 1
                    elif event.value == 0 and keycode in active_inputs:
                        del active_inputs[keycode]

                elif event.type == ecodes.EV_ABS:
                    code = ecodes.ABS[event.code]
                    self.axisChanged.emit(code, event.value)
                    
                    # Only send significant axis data to keep string short
                    if abs(event.value - 128) > 10: 
                        active_inputs[code] = event.value
                    elif code in active_inputs:
                        del active_inputs[code]
                
                # Emit current state string (e.g., "BTN_A:1, ABS_X:255")
                state_str = ", ".join([f"{k}:{v}" for k,v in active_inputs.items()])
                if not state_str: state_str = "Idle"
                self.stateUpdated.emit(state_str)

        except Exception as e:
            print(f"Input Error: {e}")

# =============================================================================
# CUSTOM WIDGETS
# =============================================================================

class RobotWidget(QWidget):
    """ Neumorphic Robot Arm """
    def __init__(self):
        super().__init__()
        self.setFixedSize(80, 100)
        self.arm_raised = False
        self.anim_angle = 0
        self.timer = QTimer()
        self.timer.timeout.connect(self.animate)
        self.timer.start(16)

    def set_arm(self, raised):
        self.arm_raised = raised

    def animate(self):
        target = -130 if self.arm_raised else 0
        self.anim_angle += (target - self.anim_angle) * 0.2
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx = w / 2
        
        body_grad = QLinearGradient(0, 0, w, h)
        body_grad.setColorAt(0, QColor("#3a3a40"))
        body_grad.setColorAt(1, QColor("#1a1a1e"))
        highlight = QColor(255, 255, 255, 30)
        shadow = QColor(0, 0, 0, 80)

        # Body
        body_rect = QRectF(cx - 20, 40, 40, 50)
        painter.setBrush(shadow); painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRoundedRect(body_rect.translated(2, 2), 15, 15)
        painter.setBrush(body_grad); painter.drawRoundedRect(body_rect, 15, 15)
        painter.setBrush(Qt.BrushStyle.NoBrush); painter.setPen(QPen(highlight, 2))
        painter.drawRoundedRect(body_rect.adjusted(1,1,-1,-1), 15, 15)

        # Head
        head_rect = QRectF(cx - 18, 10, 36, 28)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(shadow); painter.drawEllipse(head_rect.translated(2, 2))
        painter.setBrush(body_grad); painter.drawEllipse(head_rect)
        painter.setBrush(QColor(COLOR_ACCENT)); painter.drawEllipse(QPointF(cx, 24), 6, 6)
        painter.setBrush(QColor("white")); painter.drawEllipse(QPointF(cx - 2, 22), 2, 2)

        # Left Arm
        painter.setBrush(body_grad); painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRoundedRect(QRectF(cx - 32, 45, 10, 30), 5, 5)

        # Right Arm (Animated)
        painter.save()
        painter.translate(cx + 22, 50)
        painter.rotate(self.anim_angle)
        arm_rect = QRectF(-5, 0, 10, 30)
        painter.setBrush(shadow); painter.drawRoundedRect(arm_rect.translated(2,2), 5, 5)
        painter.setBrush(body_grad); painter.drawRoundedRect(arm_rect, 5, 5)
        painter.setBrush(Qt.BrushStyle.NoBrush); painter.setPen(QPen(highlight, 1))
        painter.drawRoundedRect(arm_rect.adjusted(1,1,-1,-1), 5, 5)
        painter.restore()

class DS4Button(QLabel):
    def __init__(self, text="", size=45, shape="circle", color_key="Default"):
        super().__init__(text)
        self.setFixedSize(size, size if shape == "circle" else 30)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.shape = shape
        self.active_color = BTN_COLORS.get(color_key, BTN_COLORS["Default"])
        font = self.font(); font.setPointSize(10 if len(text) > 1 else 14); font.setBold(True); self.setFont(font)
        self.pressed = False
        self.update_style()

    def set_pressed(self, pressed):
        self.pressed = pressed
        self.update_style()

    def update_style(self):
        radius = self.width() // 2 if self.shape == "circle" else 5
        if self.pressed:
            self.setStyleSheet(f"QLabel {{ background-color: {self.active_color}; color: #FFF; border: 2px solid {self.active_color}; border-radius: {radius}px; }}")
        else:
            self.setStyleSheet(f"QLabel {{ background-color: {COLOR_SURFACE}; color: {self.active_color}; border-top: 2px solid {COLOR_LIGHT}; border-left: 2px solid {COLOR_LIGHT}; border-right: 2px solid {COLOR_SHADOW}; border-bottom: 2px solid {COLOR_SHADOW}; border-radius: {radius}px; }}")

class DS4Trigger(QProgressBar):
    def __init__(self, label_text=""):
        super().__init__()
        self.setFormat(f"{label_text}")
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setFixedSize(80, 15)
        self.setTextVisible(True)
        self.setStyleSheet(f"QProgressBar {{ background-color: {COLOR_BG}; border: 1px solid {COLOR_LIGHT}; border-radius: 4px; color: {COLOR_TEXT}; font-size: 9px; }} QProgressBar::chunk {{ background-color: {COLOR_ACCENT}; border-radius: 3px; }}")

class JoystickWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.x = 0; self.y = 0
        self.setFixedSize(110, 110)
    def update_position(self, x, y):
        self.x = x; self.y = y; self.update()
    def paintEvent(self, event):
        painter = QPainter(self); painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height(); center = QPointF(w / 2, h / 2); radius = w * 0.48
        painter.setBrush(QColor("#111")); painter.setPen(QPen(QColor("#333"), 2)); painter.drawEllipse(center, radius, radius)
        stick_range = radius * 0.6; dot_x = center.x() + self.x * stick_range; dot_y = center.y() + self.y * stick_range; thumb_radius = radius * 0.6
        grad = QRadialGradient(dot_x - 5, dot_y - 5, thumb_radius * 1.5); grad.setColorAt(0, QColor("#444")); grad.setColorAt(1, QColor("#222"))
        painter.setBrush(grad); painter.setPen(Qt.PenStyle.NoPen); painter.drawEllipse(QPointF(dot_x, dot_y), thumb_radius, thumb_radius)
        painter.setBrush(QColor("#1a1a1a")); painter.drawEllipse(QPointF(dot_x, dot_y), thumb_radius * 0.7, thumb_radius * 0.7)
        painter.setBrush(Qt.BrushStyle.NoBrush); painter.setPen(QPen(QColor("#555"), 1, Qt.PenStyle.DotLine)); painter.drawEllipse(QPointF(dot_x, dot_y), thumb_radius * 0.85, thumb_radius * 0.85)

class CompactGraph(QWidget):
    def __init__(self, title, suffix="ms", max_val=100, color=COLOR_ACCENT):
        super().__init__()
        self.title = title; self.suffix = suffix; self.max_val = max_val; self.base_color = color; self.data = deque([0]*60, maxlen=60); self.setFixedHeight(50); self.current_val = 0
    def add_point(self, value): self.data.append(value); self.current_val = value; self.update()
    def paintEvent(self, event):
        painter = QPainter(self); painter.setRenderHint(QPainter.RenderHint.Antialiasing); w, h = self.width(), self.height(); painter.fillRect(self.rect(), QColor(COLOR_SURFACE))
        if not self.data: return
        path = QPainterPath(); step_x = w / (len(self.data) - 1) if len(self.data) > 1 else w; path.moveTo(0, h)
        for i, val in enumerate(self.data): x = i * step_x; norm = min(val, self.max_val) / self.max_val; y = h - (norm * (h - 5)); path.lineTo(x, y)
        path.lineTo(w, h); path.closeSubpath(); c = QColor(self.base_color); c.setAlpha(50); painter.setBrush(c); painter.setPen(Qt.PenStyle.NoPen); painter.drawPath(path)
        painter.setBrush(Qt.BrushStyle.NoBrush); painter.setPen(QPen(QColor(self.base_color), 1.5)); path_line = QPainterPath(); path_line.moveTo(0, h - (min(self.data[0], self.max_val)/self.max_val * h))
        for i, val in enumerate(self.data): x = i * step_x; y = h - (min(val, self.max_val) / self.max_val * (h - 5)); path_line.lineTo(x, y)
        painter.drawPath(path_line); painter.setPen(QColor(COLOR_TEXT_LIT)); font = painter.font(); font.setPointSize(8); painter.setFont(font); painter.drawText(QRect(5, 2, w, h), Qt.AlignmentFlag.AlignLeft, f"{self.title} {self.current_val:.1f}{self.suffix}")

# =============================================================================
# MAIN GUI
# =============================================================================

class ControllerGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Controller Dashboard")
        self.resize(650, 600)
        self.left_x = 0; self.left_y = 0; self.right_x = 0; self.right_y = 0
        self.setStyleSheet(f"background-color: {COLOR_BG}; font-family: 'Segoe UI', sans-serif;")

        main_layout = QVBoxLayout(self); main_layout.setSpacing(10); main_layout.setContentsMargins(15, 15, 15, 15)

        # --- HEADER ---
        header_layout = QHBoxLayout()
        
        # Left Triggers
        l_layout = QVBoxLayout(); l_layout.setSpacing(5)
        self.l2_bar = DS4Trigger("L2"); self.l1_btn = DS4Button("L1", size=60, shape="pill")
        l_layout.addWidget(self.l2_bar); l_layout.addWidget(self.l1_btn, alignment=Qt.AlignmentFlag.AlignCenter)
        
        # --- TOUCHPAD / DATA SCREEN ---
        touchpad_frame = QFrame(); touchpad_frame.setFixedSize(250, 90)
        touchpad_frame.setStyleSheet(f"QFrame {{ background-color: {COLOR_TOUCHPAD}; border: 1px solid {COLOR_LIGHT}; border-radius: 10px; }}")
        
        # Touchpad Split: Left (Inputs) / Right (ROS Stats)
        touch_layout = QHBoxLayout(touchpad_frame)
        touch_layout.setContentsMargins(8,5,8,5)

        # Left: Inputs
        input_data_layout = QVBoxLayout()
        self.lbl_inputs = QLabel("INPUTS:\nIdle")
        self.lbl_inputs.setStyleSheet("color: #777; font-size: 9px; font-weight: bold;")
        self.lbl_inputs.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        input_data_layout.addWidget(self.lbl_inputs)
        touch_layout.addLayout(input_data_layout, 1)

        # Vertical Divider
        div = QFrame(); div.setFrameShape(QFrame.Shape.VLine); div.setStyleSheet("color: #333;"); touch_layout.addWidget(div)

        # Right: ROS Stats (Fields required by prompt)
        ros_stats_layout = QVBoxLayout()
        self.lbl_latency = QLabel("LAT: -- ms")
        self.lbl_avg = QLabel("AVG: -- ms")
        self.lbl_loss = QLabel("LOSS: 0")
        self.lbl_stable = QLabel("STBL: -- %")
        
        style_stats = "color: #5271FF; font-family: Monospace; font-size: 10px; font-weight: bold;"
        for lbl in [self.lbl_latency, self.lbl_avg, self.lbl_loss, self.lbl_stable]:
            lbl.setStyleSheet(style_stats)
            ros_stats_layout.addWidget(lbl)
        
        touch_layout.addLayout(ros_stats_layout, 1)

        # Right Triggers
        r_layout = QVBoxLayout(); r_layout.setSpacing(5)
        self.r2_bar = DS4Trigger("R2"); self.r1_btn = DS4Button("R1", size=60, shape="pill")
        r_layout.addWidget(self.r2_bar); r_layout.addWidget(self.r1_btn, alignment=Qt.AlignmentFlag.AlignCenter)

        header_layout.addLayout(l_layout); header_layout.addStretch(); header_layout.addWidget(touchpad_frame); header_layout.addStretch(); header_layout.addLayout(r_layout)
        main_layout.addLayout(header_layout)

        # --- BODY ---
        body_layout = QHBoxLayout()
        dpad_grid = QGridLayout(); dpad_grid.setSpacing(5)
        self.dpad_btns = {"DPAD_UP": DS4Button("▲", 35), "DPAD_DOWN": DS4Button("▼", 35), "DPAD_LEFT": DS4Button("◄", 35), "DPAD_RIGHT": DS4Button("►", 35)}
        dpad_grid.addWidget(self.dpad_btns["DPAD_UP"], 0, 1); dpad_grid.addWidget(self.dpad_btns["DPAD_LEFT"], 1, 0)
        dpad_grid.addWidget(self.dpad_btns["DPAD_RIGHT"], 1, 2); dpad_grid.addWidget(self.dpad_btns["DPAD_DOWN"], 2, 1)
        body_layout.addLayout(dpad_grid)
        
        # --- CENTER (Robot + Sticks) ---
        center_stack = QVBoxLayout()
        self.robot = RobotWidget()
        robot_cont = QHBoxLayout(); robot_cont.addStretch(); robot_cont.addWidget(self.robot); robot_cont.addStretch()
        center_stack.addLayout(robot_cont)
        
        sticks_layout = QHBoxLayout(); sticks_layout.setSpacing(20)
        self.left_stick = JoystickWidget(); self.right_stick = JoystickWidget()
        sticks_layout.addWidget(self.left_stick); sticks_layout.addWidget(self.right_stick)
        center_stack.addLayout(sticks_layout)
        
        body_layout.addStretch(); body_layout.addLayout(center_stack); body_layout.addStretch()

        # Face Buttons
        face_grid = QGridLayout(); face_grid.setSpacing(5)
        self.face_btns = {"BTN_NORTH": DS4Button("▲", 40, color_key="Triangle"), "BTN_WEST": DS4Button("■", 40, color_key="Square"), "BTN_EAST": DS4Button("●", 40, color_key="Circle"), "BTN_SOUTH": DS4Button("✖", 40, color_key="Cross")}
        face_grid.addWidget(self.face_btns["BTN_NORTH"], 0, 1); face_grid.addWidget(self.face_btns["BTN_WEST"], 1, 0)
        face_grid.addWidget(self.face_btns["BTN_EAST"], 1, 2); face_grid.addWidget(self.face_btns["BTN_SOUTH"], 2, 1)
        body_layout.addLayout(face_grid)
        main_layout.addLayout(body_layout)

        # --- FOOTER GRAPHS ---
        main_layout.addSpacing(10); graph_layout = QHBoxLayout(); graph_layout.setSpacing(5)
        
        # Updated Graphs using Real ROS Data
        self.g_latency = CompactGraph("LATENCY", "ms", 100, "#E67E22") # Orange
        self.g_stability = CompactGraph("STABILITY", "%", 100, "#2ECC71") # Green
        self.g_loss = CompactGraph("PACKET LOSS", "", 20, "#E74C3C") # Red

        graph_layout.addWidget(self.g_latency)
        graph_layout.addWidget(self.g_stability)
        graph_layout.addWidget(self.g_loss)
        main_layout.addLayout(graph_layout)

        # --- LOGIC ---
        # 1. Input Thread
        self.input_thread = ControllerThread()
        self.input_thread.axisChanged.connect(self.update_axis)
        self.input_thread.buttonChanged.connect(self.update_button)
        self.input_thread.stateUpdated.connect(self.update_ros_payload)
        self.input_thread.start()

        # 2. ROS Thread
        self.ros_thread = RosWorker()
        self.ros_thread.stats_signal.connect(self.update_ros_stats)
        self.ros_thread.start()

        # Mappings
        self.all_buttons = {}; self.all_buttons.update(self.dpad_btns); self.all_buttons.update(self.face_btns); self.all_buttons["L1"] = self.l1_btn; self.all_buttons["R1"] = self.r1_btn
        self.aliases = {"BTN_TL": "L1", "BTN_TR": "R1", "BTN_A": "BTN_SOUTH", "BTN_B": "BTN_EAST", "BTN_X": "BTN_WEST", "BTN_Y": "BTN_NORTH"}

    def update_ros_payload(self, state_str):
        # Pass controller state to ROS thread to publish
        self.ros_thread.update_controller_state(state_str)
        # Update GUI text
        formatted = state_str.replace(", ", "\n")
        self.lbl_inputs.setText(f"INPUTS:\n{formatted}")

    def update_ros_stats(self, latency, avg_latency, loss, stability):
        # Update Graphs
        self.g_latency.add_point(latency)
        self.g_stability.add_point(stability)
        self.g_loss.add_point(loss)

        # Update Text Fields (Neumorphic Screen)
        self.lbl_latency.setText(f"LAT: {latency:.1f} ms")
        self.lbl_avg.setText(f"AVG: {avg_latency:.1f} ms")
        self.lbl_loss.setText(f"LOSS: {loss}")
        self.lbl_stable.setText(f"STBL: {stability:.1f} %")

        # Color code stability text
        if stability > 90: self.lbl_stable.setStyleSheet("color: #2ECC71; font-family: Monospace; font-size: 10px; font-weight: bold;")
        elif stability > 70: self.lbl_stable.setStyleSheet("color: #F1C40F; font-family: Monospace; font-size: 10px; font-weight: bold;")
        else: self.lbl_stable.setStyleSheet("color: #E74C3C; font-family: Monospace; font-size: 10px; font-weight: bold;")

    def update_button(self, code, value):
        name = self.aliases.get(code, code)
        if name in self.all_buttons: self.all_buttons[name].set_pressed(value == 1)
        if name == "BTN_NORTH": self.robot.set_arm(value == 1)

    def update_axis(self, axis, value):
        norm = (value - 128) / 128.0
        if axis == "ABS_X": self.left_x = norm; self.left_stick.update_position(self.left_x, self.left_y)
        elif axis == "ABS_Y": self.left_y = norm; self.left_stick.update_position(self.left_x, self.left_y)
        elif axis == "ABS_RX": self.right_x = norm; self.right_stick.update_position(self.right_x, self.right_y)
        elif axis == "ABS_RY": self.right_y = norm; self.right_stick.update_position(self.right_x, self.right_y)
        elif axis == "ABS_Z": self.l2_bar.setValue(value)
        elif axis == "ABS_RZ": self.r2_bar.setValue(value)
        elif axis == "ABS_HAT0X": self.dpad_btns["DPAD_LEFT"].set_pressed(value == -1); self.dpad_btns["DPAD_RIGHT"].set_pressed(value == 1)
        elif axis == "ABS_HAT0Y": self.dpad_btns["DPAD_UP"].set_pressed(value == -1); self.dpad_btns["DPAD_DOWN"].set_pressed(value == 1)

    def closeEvent(self, event):
        self.input_thread.terminate()
        self.ros_thread.stop()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ControllerGUI()
    window.show()
    sys.exit(app.exec())