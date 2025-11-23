#!/usr/bin/env python3
import sys
import time
import os
import json
from collections import deque
import numpy as np
import cv2

# PyQt6 Imports
from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *
from PyQt6.QtSvg import QSvgRenderer

# Input Imports
from evdev import InputDevice, categorize, ecodes, list_devices



os.system('bash -c "source /opt/ros/humble/setup.bash && echo setup complete"')


# ROS2 Imports
from cv_bridge import CvBridge
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Image as RosImage # Renamed to avoid conflict with PyQt 'Image'
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# =============================================================================
# CONSTANTS & CONFIG
# =============================================================================

MAPPING_FILE = "controller_mappings.json"
STICK_DEADZONE = 0.10
AXIS_DETECT_THRESHOLD = 0.5 # Increased to 50% to prevent accidental nudges

# Colors
COLOR_BG        = "#1A1A1D"
COLOR_SURFACE   = "#2C2C31"
COLOR_TOUCHPAD  = "#151518"
COLOR_LIGHT     = "#484850"
COLOR_ACCENT    = "#5271FF"
COLOR_SUCCESS   = "#2ECC71"
COLOR_WARNING   = "#F1C40F"
COLOR_TEXT_LIT  = "#FFFFFF"

# =============================================================================
# MAPPING TEMPLATE
# =============================================================================

LOGICAL_MAP_TEMPLATE = [
    ("L_STICK_X", "Move Left Stick LEFT/RIGHT", "axis_full"),
    ("L_STICK_Y", "Move Left Stick UP/DOWN", "axis_full"),
    ("R_STICK_X", "Move Right Stick LEFT/RIGHT", "axis_full"),
    ("R_STICK_Y", "Move Right Stick UP/DOWN", "axis_full"),
    
    ("L2_INPUT",  "Press L2 Trigger", "any"),
    ("R2_INPUT",  "Press R2 Trigger", "any"),
    
    ("DPAD_UP",   "Press D-Pad UP", "btn_discrete"),
    ("DPAD_DOWN", "Press D-Pad DOWN", "btn_discrete"),
    ("DPAD_LEFT", "Press D-Pad LEFT", "btn_discrete"),
    ("DPAD_RIGHT","Press D-Pad RIGHT", "btn_discrete"),
    
    ("BTN_SOUTH", "Press Bottom Face Button (X/A)", "btn_discrete"),
    ("BTN_EAST",  "Press Right Face Button (O/B)", "btn_discrete"),
    ("BTN_WEST",  "Press Left Face Button (Sqr/X)", "btn_discrete"),
    ("BTN_NORTH", "Press Top Face Button (Tri/Y)", "btn_discrete"),
    ("BTN_L1",    "Press L1 Bumper", "btn_discrete"),
    ("BTN_R1",    "Press R1 Bumper", "btn_discrete"),
    ("BTN_L3",    "Press LEFT Stick Down (L3)", "btn_discrete"),
    ("BTN_R3",    "Press RIGHT Stick Down (R3)", "btn_discrete"),
]

# =============================================================================
# SMART MAPPING WIZARD
# =============================================================================

class MappingWorker(QThread):
    state_update = pyqtSignal(dict)
    calibration_progress = pyqtSignal(int) 

    def __init__(self, device_path):
        super().__init__()
        self.device_path = device_path
        self.running = True
        
        self.current_state = {} 
        self.baseline_state = {} 
        self.is_calibrated = False

    def _normalize(self, val, info):
        """Helper to safely normalize any axis to -1.0 ... 1.0"""
        rng = info.max - info.min
        if rng == 0: rng = 1
        return 2 * (val - info.min) / rng - 1.0

    def run(self):
        try:
            dev = InputDevice(self.device_path)
            
            # --- PHASE 1: FORCE READ HARDWARE STATE ---
            # We do NOT wait for an event. We query the driver directly.
            
            # 1. Initialize Buttons to 0
            for code in dev.capabilities().get(ecodes.EV_KEY, []):
                self.current_state[code] = {"type": "btn", "value": 0.0}
                self.baseline_state[code] = 0.0 

            # 2. Initialize Axes (CRITICAL FIX)
            # We query 'absinfo' to get the current PHYSICAL position immediately.
            # This catches Triggers resting at -1.0 before they are pressed.
            for code in dev.capabilities().get(ecodes.EV_ABS, []):
                try:
                    info = dev.absinfo(code)
                    val = self._normalize(info.value, info)
                    
                    self.current_state[code] = {"type": "axis", "value": val}
                    self.baseline_state[code] = val 
                except: 
                    pass

            # --- PHASE 2: CALIBRATION (1 Second) ---
            # We verify the resting rate. If values jitter, we update the baseline.
            samples = 25
            for i in range(samples):
                if not self.running: return
                
                # Check for new events to update baseline
                while True:
                    try:
                        event = dev.read_one()
                        if not event: break
                        if event.type == ecodes.EV_ABS:
                            info = dev.absinfo(event.code)
                            val = self._normalize(event.value, info)
                            self.baseline_state[event.code] = val 
                    except BlockingIOError: break
                
                self.calibration_progress.emit(int((i / samples) * 100))
                QThread.msleep(40) 

            self.is_calibrated = True
            self.calibration_progress.emit(100)

            # --- PHASE 3: RUNTIME LOOP ---
            while self.running:
                batch_end = time.time() + 0.02 
                while time.time() < batch_end:
                    try:
                        event = dev.read_one()
                        if not event: break
                        
                        if event.type == ecodes.EV_KEY:
                            self.current_state[event.code] = {"type": "btn", "value": float(event.value)}
                        
                        elif event.type == ecodes.EV_ABS:
                            try:
                                info = dev.absinfo(event.code)
                                val = self._normalize(event.value, info)
                                self.current_state[event.code] = {"type": "axis", "value": val}
                            except: pass
                    except BlockingIOError: break

                packet = {
                    "current": self.current_state,
                    "baseline": self.baseline_state
                }
                self.state_update.emit(packet)
                QThread.msleep(10)
                
        except Exception as e:
            print(f"Mapping Thread Error: {e}")

    def stop(self):
        self.running = False

class MappingWizard(QDialog):
    def __init__(self, device_path, device_name):
        super().__init__()
        self.setWindowTitle(f"Map Controller: {device_name}")
        self.resize(550, 450)
        self.setStyleSheet(f"background-color: {COLOR_BG}; color: {COLOR_TEXT_LIT}; font-family: 'Segoe UI';")
        
        self.current_step = 0
        self.mapping_result = {"device_name": device_name, "map": {}}

        # --- Stability Logic ---
        self.stability_counter = 0
        # Wait approx 250ms (25 frames * 10ms) for input to settle before unlocking
        self.REQUIRED_STABLE_FRAMES = 25 

        layout = QVBoxLayout(self)
        self.lbl_step = QLabel("Initializing...")
        self.lbl_step.setStyleSheet("color: #777; font-weight:bold;")
        layout.addWidget(self.lbl_step)

        self.lbl_instruction = QLabel("Don't touch the controller!\nCalibrating resting points...")
        self.lbl_instruction.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_instruction.setStyleSheet(f"font-size: 20px; color: {COLOR_WARNING}; font-weight: bold; margin: 20px;")
        layout.addWidget(self.lbl_instruction)
        
        self.lbl_debug = QLabel("")
        self.lbl_debug.setStyleSheet("color: #555; font-size: 10px; font-family: monospace;")
        self.lbl_debug.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.lbl_debug)

        self.pbar = QProgressBar()
        self.pbar.setRange(0, 100)
        self.pbar.setStyleSheet(f"QProgressBar::chunk {{ background-color: {COLOR_ACCENT}; }}")
        layout.addWidget(self.pbar)

        self.list_preview = QListWidget()
        self.list_preview.setStyleSheet(f"background-color: {COLOR_SURFACE}; border: none;")
        layout.addWidget(self.list_preview)

        self.btn_skip = QPushButton("Skip This Input")
        self.btn_skip.clicked.connect(self.skip_step)
        self.btn_skip.hide()

        self.btn_start = QPushButton("Start Mapping")
        self.btn_start.clicked.connect(self.start_mapping)
        self.btn_start.setStyleSheet(f"background-color: {COLOR_SUCCESS}; padding: 10px; font-weight: bold; color:white;")
        self.btn_start.setEnabled(False) 
        
        layout.addWidget(self.btn_skip)
        layout.addWidget(self.btn_start)

        self.worker = MappingWorker(device_path)
        self.worker.state_update.connect(self.process_state_snapshot)
        self.worker.calibration_progress.connect(self.update_calibration)
        
        self.worker.start()
        self.listening = False
        self.wait_for_neutral = True 

    def update_calibration(self, val):
        self.pbar.setValue(val)
        if val >= 100:
            self.lbl_instruction.setText("Calibration Complete.\nPress Start to map.")
            self.lbl_instruction.setStyleSheet(f"font-size: 20px; color: {COLOR_SUCCESS}; font-weight: bold; margin: 20px;")
            self.btn_start.setEnabled(True)
            self.pbar.setRange(0, len(LOGICAL_MAP_TEMPLATE))
            self.pbar.setValue(0)

    def start_mapping(self):
        self.listening = True
        self.wait_for_neutral = True 
        self.stability_counter = 0 
        self.btn_start.hide()
        self.btn_skip.show()
        self.update_ui()

    def update_ui(self):
        if self.current_step >= len(LOGICAL_MAP_TEMPLATE):
            self.finish_mapping()
            return
        
        log, txt, typ = LOGICAL_MAP_TEMPLATE[self.current_step]
        self.lbl_step.setText(f"Step {self.current_step + 1}/{len(LOGICAL_MAP_TEMPLATE)}")
        self.pbar.setValue(self.current_step)
        
        self.wait_for_neutral = True 
        self.stability_counter = 0 
        self.lbl_instruction.setStyleSheet(f"font-size: 20px; color: {COLOR_WARNING}; font-weight: bold; margin: 20px;")
        self.lbl_instruction.setText(f"{txt}\n(Release all buttons...)")

    def process_state_snapshot(self, packet):
        if not self.worker.is_calibrated or not self.listening: return
        if self.current_step >= len(LOGICAL_MAP_TEMPLATE): return
        
        full_state = packet["current"]
        baseline = packet["baseline"]

        # --- 1. NEUTRALITY CHECK ---
        is_currently_silent = True
        offending_input = ""
        
        for code, data in full_state.items():
            if data["type"] == "btn":
                if data["value"] == 1: 
                    is_currently_silent = False
                    offending_input = f"Btn {code}"
                    break
            elif data["type"] == "axis":
                base_val = baseline.get(code, 0.0)
                curr_val = data["value"]
                
                # TOLERANCE: 0.35 allows loose triggers to drift slightly without breaking neutral
                if abs(curr_val - base_val) > 0.35:
                    is_currently_silent = False
                    offending_input = f"Axis {code} ({curr_val:.2f})"
                    break
        
        # --- 2. LOCK/UNLOCK LOGIC ---
        if self.wait_for_neutral:
            if is_currently_silent:
                self.stability_counter += 1
            else:
                self.stability_counter = 0 # Reset on any noise
                self.lbl_debug.setText(f"Wait for Release: {offending_input}")

            if self.stability_counter > self.REQUIRED_STABLE_FRAMES:
                self.wait_for_neutral = False
                self.lbl_debug.setText("Status: Listening")
                log, txt, typ = LOGICAL_MAP_TEMPLATE[self.current_step]
                self.lbl_instruction.setText(txt)
                self.lbl_instruction.setStyleSheet(f"font-size: 20px; color: {COLOR_ACCENT}; font-weight: bold; margin: 20px;")
            return 

        # --- 3. INPUT DETECTION ---
        best_code = None
        best_data = None
        max_strength = 0

        for code, data in full_state.items():
            strength = 0
            if data["type"] == "btn": 
                strength = data["value"]
            elif data["type"] == "axis":
                base_val = baseline.get(code, 0.0)
                strength = abs(data["value"] - base_val)
            
            if strength > max_strength:
                max_strength = strength
                best_code = code
                best_data = data

        if best_code is not None:
            target_log, _, target_type = LOGICAL_MAP_TEMPLATE[self.current_step]
            valid = False
            map_entry = {}
            
            # ACTIVATION THRESHOLD: 0.6
            # You must press the trigger 60% of the way down to register.
            # This prevents small "bounce back" jitters from registering.
            ACTIVATE_THRESH = 0.6

            if target_type == "axis_full":
                if best_data["type"] == "axis" and max_strength > ACTIVATE_THRESH:
                    map_entry = {"logical": target_log, "type": "axis"}
                    valid = True
            
            elif target_type == "btn_discrete":
                if best_data["type"] == "btn" and best_data["value"] == 1:
                    map_entry = {"logical": target_log, "type": "btn"}
                    valid = True
                elif best_data["type"] == "axis" and max_strength > 0.8:
                    base_val = baseline.get(best_code, 0.0)
                    val = best_data["value"]
                    direction = 1 if (val - base_val) > 0 else -1
                    map_entry = {"logical": target_log, "type": "axis_as_btn", "trigger_val": direction}
                    valid = True

            elif target_type == "any":
                if best_data["type"] == "btn" and best_data["value"] == 1:
                    map_entry = {"logical": target_log, "type": "btn"}
                    valid = True
                elif best_data["type"] == "axis" and max_strength > ACTIVATE_THRESH:
                    map_entry = {"logical": target_log, "type": "axis"}
                    valid = True

            if valid:
                self.mapping_result["map"][str(best_code)] = map_entry
                self.list_preview.addItem(f"Mapped {target_log} -> Code {best_code}")
                self.list_preview.scrollToBottom()
                
                self.current_step += 1
                self.wait_for_neutral = True 
                self.stability_counter = 0 
                QTimer.singleShot(100, self.update_ui) 

    def skip_step(self):
        self.current_step += 1
        self.update_ui()

    def finish_mapping(self):
        self.listening = False
        self.worker.stop()
        self.accept()

# =============================================================================
# INPUT PROCESSOR (RUNTIME)
# =============================================================================

class ControllerThread(QThread):
    inputEvent = pyqtSignal(str, float) 
    stateUpdated = pyqtSignal(str) 
    stateDictUpdated = pyqtSignal(dict)

    def __init__(self, device_path, profile):
        super().__init__()
        self.device_path = device_path
        self.profile = profile 
        self.running = True

    def run(self):
        try:
            gamepad = InputDevice(self.device_path)
            
            axis_calib = {}
            for code_str, map_data in self.profile.items():
                if "axis" in map_data["type"]:
                    code = int(code_str)
                    try:
                        info = gamepad.absinfo(code)
                        axis_calib[code] = {"min": info.min, "max": info.max}
                    except:
                        axis_calib[code] = {"min": 0, "max": 255}

            active_states = {} 

            for event in gamepad.read_loop():
                if not self.running: break
                
                code_str = str(event.code)
                if code_str not in self.profile: continue
                
                mapping = self.profile[code_str]
                logical = mapping["logical"]
                m_type = mapping["type"]
                has_changed = False

                if m_type == "btn" and event.type == ecodes.EV_KEY:
                    val = float(event.value)
                    self.inputEvent.emit(logical, val)
                    if val: active_states[logical] = 1
                    elif logical in active_states: del active_states[logical]
                    has_changed = True

                elif event.type == ecodes.EV_ABS:
                    # 1. Normalize to -1.0 to 1.0 based on driver min/max
                    cal = axis_calib[int(code_str)]
                    rng = cal["max"] - cal["min"]
                    if rng == 0: rng = 1
                    
                    # This standardizes all axes (sticks and triggers) to -1.0 ... 1.0
                    norm = 2 * (event.value - cal["min"]) / rng - 1.0
                    
                    if m_type == "axis":
                        # === FIX: L2/R2 TRIGGER LOGIC ===
                        if logical in ["L2_INPUT", "R2_INPUT"]:
                            # Triggers usually rest at -1.0 and press to 1.0.
                            # We want to map this range to 0.0 ... 1.0.
                            # Formula: (Input + 1) / 2
                            norm = (norm + 1.0) / 2.0
                            
                            # Clamp: Ensure we don't go below 0 (noise) or above 1
                            norm = max(0.0, min(1.0, norm))

                        # Apply Deadzone
                        if abs(norm) < STICK_DEADZONE: norm = 0.0
                        
                        self.inputEvent.emit(logical, norm)
                        
                        if abs(norm) > 0: 
                            active_states[logical] = round(norm, 2)
                        elif logical in active_states: 
                            del active_states[logical]
                        has_changed = True
                    
                    elif m_type == "axis_as_btn":
                        # D-Pad Splitter logic
                        pair_map = {
                            "DPAD_DOWN":  ("DPAD_UP", "DPAD_DOWN"),
                            "DPAD_UP":    ("DPAD_UP", "DPAD_DOWN"),
                            "DPAD_RIGHT": ("DPAD_LEFT", "DPAD_RIGHT"),
                            "DPAD_LEFT":  ("DPAD_LEFT", "DPAD_RIGHT")
                        }

                        if logical in pair_map:
                            neg_btn, pos_btn = pair_map[logical]
                            is_neg_pressed = (norm < -0.5)
                            is_pos_pressed = (norm > 0.5)

                            val_neg = 1.0 if is_neg_pressed else 0.0
                            self.inputEvent.emit(neg_btn, val_neg)
                            if val_neg: active_states[neg_btn] = 1
                            elif neg_btn in active_states: 
                                if neg_btn in active_states: del active_states[neg_btn]

                            val_pos = 1.0 if is_pos_pressed else 0.0
                            self.inputEvent.emit(pos_btn, val_pos)
                            if val_pos: active_states[pos_btn] = 1
                            elif pos_btn in active_states: 
                                if pos_btn in active_states: del active_states[pos_btn]
                        else:
                            target = mapping["trigger_val"]
                            is_pressed = False
                            if target < 0 and norm < -0.5: is_pressed = True
                            if target > 0 and norm > 0.5: is_pressed = True
                            val = 1.0 if is_pressed else 0.0
                            self.inputEvent.emit(logical, val)
                            if val: active_states[logical] = 1
                            elif logical in active_states: del active_states[logical]
                        has_changed = True

                # Publish Logic
                state_str = ", ".join([f"{k}:{v}" for k,v in active_states.items()])
                if not state_str: state_str = "Idle"
                self.stateUpdated.emit(state_str)
                
                if has_changed or len(active_states) == 0:
                    self.stateDictUpdated.emit(active_states.copy())

        except Exception as e:
            print(f"Controller Error: {e}")

    def stop(self):
        self.running = False

# =============================================================================
# ROS2 WORKER (UPDATED FOR 10HZ TIMER)
# =============================================================================

class RosWorker(QThread):
    stats_signal = pyqtSignal(float, float, int, float, bool)
    video_signal = pyqtSignal(QImage)
    
    def __init__(self):
        super().__init__()
        self.active = True
        self.node = None 

    def send_controller_state(self, state_dict):
        if self.node:
            self.node.update_input_cache(state_dict)

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
        self.msg_id = 0
        self.sent_t = {}
        self.lats = deque(maxlen=20)
        
        self.rx = 0
        self.tx = 0
        self.last_rx_time = 0
        self.current_rate = 0.0
        
        # --- CAMERA STATE DEFAULT: OFF ---
        self.camera_enabled = False 

    def update_input_cache(self, inputs_dict):
        self.current_inputs = inputs_dict

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_img.shape
            bytes_per_line = ch * w
            qt_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
            self.worker.video_signal.emit(qt_img)
        except Exception:
            pass

    def timer_tick(self):
        is_connected = (time.time() - self.last_rx_time) < 1.0
        
        if not is_connected:
            self.current_rate = 0.0; lat = 0.0; stbl = 0.0
        else:
            lat = self.lats[-1] if self.lats else 0
            stbl = (self.rx/self.tx)*100 if self.tx > 0 else 0

        loss = self.tx - self.rx
        self.worker.stats_signal.emit(float(lat), float(stbl), int(loss), float(self.current_rate), is_connected)

        # Send Packet with Camera Flag
        packet = {
            "id": self.msg_id,
            "timestamp": time.time(),
            "inputs": self.current_inputs,
            "camera_enabled": self.camera_enabled 
        }
        
        msg = String()
        msg.data = json.dumps(packet)
        self.pub.publish(msg)
        
        self.sent_t[self.msg_id] = time.time()
        self.tx += 1
        self.msg_id += 1

    def callback(self, msg):
        try:
            now = time.time()
            data = json.loads(msg.data)
            mid = data.get("id", -1)
            
            if mid in self.sent_t:
                lat = (now - self.sent_t[mid]) * 1000
                del self.sent_t[mid]
                
                if self.last_rx_time > 0:
                    delta = now - self.last_rx_time
                    if delta > 0:
                        inst_rate = 1.0 / delta
                        self.current_rate = (self.current_rate * 0.7) + (inst_rate * 0.3)
                
                self.last_rx_time = now
                self.lats.append(lat)
                self.rx += 1
        except: 
            pass

# =============================================================================
# GUI CLASSES
# =============================================================================

# Include DeviceSelectionDialog, SimpleTrigger, CompactGraph, ControllerVisualizer 
# (These remain the same as Part 2, omitted here for brevity, assume they exist)
# PASTE THE CLASSES FROM PREVIOUS RESPONSE IF CREATING NEW FILE

class DeviceSelectionDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Select Controller")
        self.setFixedSize(450, 400)
        self.setStyleSheet(f"background-color: {COLOR_BG}; color: {COLOR_TEXT_LIT}; font-family: 'Segoe UI';")
        self.selected_path = None
        self.selected_profile = None

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("1. Select Hardware Device:"))
        self.list_widget = QListWidget()
        self.list_widget.setStyleSheet(f"""
            QListWidget {{ background-color: {COLOR_SURFACE}; border: 1px solid {COLOR_LIGHT}; }}
            QListWidget::item:selected {{ background-color: {COLOR_ACCENT}; }}
        """)
        # Prevent default focus/highlight on list
        self.list_widget.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        layout.addWidget(self.list_widget)

        layout.addWidget(QLabel("2. Controller Mapping:"))
        btn_layout = QHBoxLayout()
        
        self.btn_map = QPushButton("Create New Mapping")
        self.btn_map.setStyleSheet(f"background-color: {COLOR_LIGHT}; padding: 5px;")
        self.btn_map.setEnabled(False) 
        self.btn_map.clicked.connect(self.open_mapper)
        
        self.lbl_status = QLabel("No Selection"); self.lbl_status.setStyleSheet("color:#777")
        btn_layout.addWidget(self.btn_map); btn_layout.addWidget(self.lbl_status)
        layout.addLayout(btn_layout)

        self.connect_btn = QPushButton("Connect"); self.connect_btn.setEnabled(False)
        self.connect_btn.setStyleSheet(f"background-color: {COLOR_ACCENT}; padding:10px; font-weight:bold; border:none")
        self.connect_btn.clicked.connect(self.accept_selection)
        layout.addWidget(self.connect_btn)

        self.device_map = {}
        self.refresh_devices()
        
        self.list_widget.itemSelectionChanged.connect(self.on_selection_change)

    def refresh_devices(self):
        self.list_widget.clear()
        try:
            for i, path in enumerate(list_devices()):
                dev = InputDevice(path)
                if ecodes.EV_ABS in dev.capabilities():
                    self.list_widget.addItem(f"{dev.name}\n{dev.phys}")
                    self.device_map[i] = dev
        except: pass
        
        # FIX: Ensure no item is selected or highlighted by default
        self.list_widget.clearSelection()
        self.list_widget.setCurrentRow(-1)

    def on_selection_change(self):
        self.list_widget.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        
        row = self.list_widget.currentRow()
        if row in self.device_map:
            self.btn_map.setEnabled(True)
            self.btn_map.setStyleSheet(f"background-color: {COLOR_ACCENT}; color: white; padding: 5px;")
            self.check_profile()
        else:
            self.btn_map.setEnabled(False)
            self.btn_map.setStyleSheet(f"background-color: {COLOR_LIGHT}; padding: 5px;")

    def check_profile(self):
        row = self.list_widget.currentRow()
        if row not in self.device_map: return
        dev = self.device_map[row]
        
        if os.path.exists(MAPPING_FILE):
            with open(MAPPING_FILE, 'r') as f:
                data = json.load(f)
                if dev.name in data:
                    self.selected_profile = data[dev.name]
                    self.lbl_status.setText("Profile Found ✓"); self.lbl_status.setStyleSheet(f"color:{COLOR_SUCCESS}")
                    self.connect_btn.setEnabled(True)
                    return
        self.lbl_status.setText("Profile Missing"); self.lbl_status.setStyleSheet(f"color:{COLOR_WARNING}")
        self.connect_btn.setEnabled(False)

    def open_mapper(self):
        row = self.list_widget.currentRow()
        if row not in self.device_map: return
        dev = self.device_map[row]
        wiz = MappingWizard(dev.path, dev.name)
        if wiz.exec() == QDialog.DialogCode.Accepted:
            full = {}
            if os.path.exists(MAPPING_FILE):
                with open(MAPPING_FILE,'r') as f: full = json.load(f)
            full[dev.name] = wiz.mapping_result["map"]
            with open(MAPPING_FILE,'w') as f: json.dump(full, f, indent=4)
            self.check_profile()

    def accept_selection(self):
        row = self.list_widget.currentRow()
        self.selected_path = self.device_map[row].path
        self.accept()

# Trigger/Progress Bar (Used for L2/R2)
class SimpleTrigger(QProgressBar):
    def __init__(self, txt): 
        super().__init__(); 
        self.setFormat(txt); 
        self.setAlignment(Qt.AlignmentFlag.AlignCenter); 
        self.setStyleSheet(f"QProgressBar {{ background:{COLOR_SURFACE}; border:1px solid {COLOR_LIGHT}; color:{COLOR_TEXT_LIT}; font-size:10px; }} QProgressBar::chunk {{ background:{COLOR_ACCENT}; }}")
        self.setTextVisible(True)

# CompactGraph (Used for ROS data)
class CompactGraph(QWidget):
    def __init__(self, t, s, m, c):
        super().__init__(); self.t=t; self.s=s; self.m=m; self.c=c; self.d=deque([0]*60, maxlen=60); self.setFixedHeight(50); self.cur=0
    def add(self, v): self.d.append(v); self.cur=v; self.update()
    def paintEvent(self, e):
        p = QPainter(self); p.fillRect(self.rect(), QColor(COLOR_SURFACE)); w=self.width(); h=self.height()
        if not self.d: return
        path = QPainterPath(); path.moveTo(0, h)
        step = w/(len(self.d)-1) if len(self.d)>1 else w
        for i, v in enumerate(self.d): path.lineTo(i*step, h - (min(v, self.m)/self.m * (h-5)))
        p.setPen(QPen(QColor(self.c), 1.5)); p.drawPath(path)
        p.setPen(QColor("#FFF")); p.drawText(5, 12, f"{self.t} {self.cur:.1f}{self.s}")

# ControllerVisualizer (The core drawing widget)
class ControllerVisualizer(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(320, 220) 
        
        # Color Configuration
        self.col_off  = "#222222"    # Unpressed button (dark grey)
        self.col_on   = "#5271FF"    # Pressed button / Slider Fill (Bright Blue)
        self.col_body = "#444444"    # Controller Shell
        self.col_trig_bg = "#333333" # Trigger background track

        # Trigger Slider State
        self.max_trig_w = 35.0 # Maximum width defined in SVG geometry
        self.trig_l_w = 0.001  # Start near zero width
        self.trig_r_w = 0.001
        
        # State tracking for standard on/off buttons
        self.colors = {
            "BTN_SOUTH": self.col_off, "BTN_EAST": self.col_off,
            "BTN_WEST": self.col_off,  "BTN_NORTH": self.col_off,
            "BTN_L1": self.col_off,    "BTN_R1": self.col_off,
            "BTN_L3": self.col_off,    "BTN_R3": self.col_off,
            "DPAD_UP": self.col_off,   "DPAD_DOWN": self.col_off,
            "DPAD_LEFT": self.col_off, "DPAD_RIGHT": self.col_off,
            # L2/R2 removed from here as they are handled by width now
        }
        
        # Joystick Offsets (X, Y)
        self.joy_l = (0, 0)
        self.joy_r = (0, 0)

        self.reload_svg()

    def get_svg_template(self):
        # We use f-string interpolation to inject colors and widths dynamically.
        return f"""
        <svg viewBox="-10 820 280 190" xmlns="http://www.w3.org/2000/svg">
            <path id="shell" 
                  d="m54.241 842.41c-0.47597-0.007-0.95982 0.005-1.438 0.0167-3.8255 0.097-7.7038 0.77663-11.333 1.6092-3.3992 0.77986-7.0506 2.0811-9.3643 4.6907-3.6256 4.0892-5.5124 15.425-5.5124 15.425s-10.619 7.0994-14.329 12.138c-3.4896 4.7395-5.3216 10.576-6.8991 16.246-1.0445 3.7543-1.291 7.6918-1.6435 11.573-2.3278 25.63-5.0152 48.958-3.0301 77.157 0.70842 10.064 5.0934 14.999 11.025 19.567 5.4892 4.2276 13.256 6.215 20.115 5.2385 5.3841-0.7665 10.178-4.4895 14.004-8.3542 7.9784-8.0589 15.483-28.016 17.137-29.394 1.6533-1.3777 3.8519-4.6907 3.8519-4.6907s5.2928 4.7838 8.5426 6.0603c4.5704 1.7951 9.7365 2.5903 14.603 1.9345 6.323-0.85208 12.786-3.2986 17.633-7.4469 3.447-2.95 7.173-11.573 7.173-11.573h26.449s3.726 8.6227 7.173 11.573c4.8474 4.1484 11.31 6.5948 17.633 7.4469 4.8663 0.65579 10.032-0.13934 14.603-1.9345 3.2498-1.2764 8.5426-6.0603 8.5426-6.0603s2.1986 3.313 3.8518 4.6907c1.6533 1.3778 9.141 21.335 17.119 29.394 3.8262 3.8648 8.6367 7.5878 14.021 8.3542 6.8593 0.9765 14.626-1.0109 20.115-5.2385 5.9315-4.5682 10.316-9.5038 11.025-19.567 1.985-28.199-0.70235-51.527-3.0301-77.157-0.35248-3.8809-0.61614-7.8184-1.6606-11.573-1.5775-5.6702-3.3924-11.507-6.882-16.246-3.7096-5.0382-14.329-12.138-14.329-12.138s-1.9041-11.335-5.5296-15.425c-2.3137-2.6096-5.9479-3.9109-9.3472-4.6907-7.2584-1.6652-15.525-2.7085-22.341 0.29103-3.9204 1.7252-6.595 6.0577-8.7994 9.3643s-3.304 13.781-3.304 13.781l-38.313-0.15416v-0.11974l-13.764 0.0514-13.781-0.0514v0.11974l-38.313 0.15416s-1.0996-10.475-3.304-13.781c-2.2044-3.3066-4.879-7.6391-8.7994-9.3643-2.9821-1.3123-6.2379-1.8667-9.5698-1.9174zm155.5 27.511zm-171.31 5.307zm148.55 14.62zm45.538 0zm-208.86 0.68478zm33.417 0zm88.705 4.023 14.687 4.8106-14.687 4.8277zm-51.345 0.33876h14.633v8.9607h-14.6zm-48.453 7.7245zm163.76 7.17zm-123.66 16.4c10.12 0 18.318 8.1976 18.318 18.318s-8.1976 18.335-18.318 18.335-18.318-8.2148-18.318-18.335 8.1977-18.318 18.318-18.318zm82.721 0c10.12 0 18.318 8.1976 18.318 18.318s-8.1976 18.335-18.318 18.335-18.318-8.2148-18.318-18.335 8.1976-18.318 18.318-18.318z" 
                  fill="{self.col_body}" 
                  stroke="#222" 
                  stroke-width="2" 
                  fill-rule="evenodd" />
            
            <rect x="33" y="850" width="35" height="15" rx="5" fill="{self.colors['BTN_L1']}" />
            <rect x="188" y="850" width="35" height="15" rx="5" fill="{self.colors['BTN_R1']}" />

            <rect x="33" y="822" width="35" height="15" rx="3" fill="{self.col_trig_bg}" />
            <rect x="33" y="822" width="{self.trig_l_w}" height="15" rx="3" fill="{self.col_on}" />

            <rect x="188" y="822" width="35" height="15" rx="3" fill="{self.col_trig_bg}" />
            <rect x="188" y="822" width="{self.trig_r_w}" height="15" rx="3" fill="{self.col_on}" />

            <g transform="translate(46, 898)"> 
                <path d="M0 -2 L9 -11 L9 -25 L-9 -25 L-9 -11 Z" fill="{self.colors['DPAD_UP']}" />
                <path d="M0 2 L9 11 L9 25 L-9 25 L-9 11 Z" fill="{self.colors['DPAD_DOWN']}" />
                <path d="M-2 0 L-11 9 L-25 9 L-25 -9 L-11 -9 Z" fill="{self.colors['DPAD_LEFT']}" />
                <path d="M2 0 L11 9 L25 9 L25 -9 L11 -9 Z" fill="{self.colors['DPAD_RIGHT']}" />
            </g>

            <g transform="translate(210, 905)"> 
                <circle cx="0" cy="-20" r="10" fill="{self.colors['BTN_NORTH']}" />
                <circle cx="0" cy="20" r="10" fill="{self.colors['BTN_SOUTH']}" />
                <circle cx="-20" cy="0" r="10" fill="{self.colors['BTN_WEST']}" />
                <circle cx="20" cy="0" r="10" fill="{self.colors['BTN_EAST']}" />
            </g>

            <g transform="translate(86, 945)">
                <circle cx="0" cy="0" r="18" fill="#111" /> 
                <circle cx="{self.joy_l[0]}" cy="{self.joy_l[1]}" r="15" fill="{self.colors['BTN_L3']}" stroke="#333" stroke-width="2"/>
            </g>

            <g transform="translate(169, 945)">
                <circle cx="0" cy="0" r="18" fill="#111" />
                <circle cx="{self.joy_r[0]}" cy="{self.joy_r[1]}" r="15" fill="{self.colors['BTN_R3']}" stroke="#333" stroke-width="2"/>
            </g>
        </svg>
        """

    def reload_svg(self):
        svg_content = self.get_svg_template()
        self.renderer = QSvgRenderer(QByteArray(svg_content.encode('utf-8')))
        self.update()

    def set_input(self, key, val):
        # 1. Handle Analog Triggers (Calculate Width)
        if key == "L2_INPUT":
            # Calculate width: input (0.0-1.0) * max width (35)
            # Use max(0.001, ...) to ensure it doesn't disappear completely at 0
            self.trig_l_w = max(0.001, val * self.max_trig_w)
        elif key == "R2_INPUT":
            self.trig_r_w = max(0.001, val * self.max_trig_w)

        # 2. Handle Standard Buttons (Boolean Color toggle)
        elif key in self.colors:
            is_active = val > 0.4
            self.colors[key] = self.col_on if is_active else self.col_off

        # 3. Handle Joysticks (Calculate Offset)
        scale = 8.0
        if key == "L_STICK_X": self.joy_l = (val * scale, self.joy_l[1])
        elif key == "L_STICK_Y": self.joy_l = (self.joy_l[0], val * scale)
        elif key == "R_STICK_X": self.joy_r = (val * scale, self.joy_r[1])
        elif key == "R_STICK_Y": self.joy_r = (self.joy_r[0], val * scale)

        self.reload_svg()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        if self.renderer.isValid():
            self.renderer.render(p, QRectF(self.rect()))
        else:
            # Fallback if SVG breaks
            p.setBrush(QColor("red"))
            p.drawRect(self.rect())


class ControllerGUI(QWidget):
    def __init__(self, path, profile):
        super().__init__()
        self.setWindowTitle("ROS2 Controller Dashboard")
        self.setStyleSheet(f"background:{COLOR_BG}; font-family:'Segoe UI'; color:{COLOR_TEXT_LIT};")
        self.resize(1100, 750) 
        
        main_layout = QHBoxLayout(self)
        main_layout.setSpacing(25)
        main_layout.setContentsMargins(25, 25, 25, 25)
        
        # LEFT PANEL
        left_panel = QVBoxLayout()
        left_panel.setSpacing(10)
        
        left_panel.addWidget(self._create_heading("📡 Connection Status"))
        
        self.lbl_conn = QLabel("● DISCONNECTED")
        self.lbl_conn.setStyleSheet(f"color: {COLOR_WARNING}; font-weight: bold; font-size: 16px; margin-bottom: 5px;")
        left_panel.addWidget(self.lbl_conn)
        
        self.lbl_loss = QLabel("Packet Loss: 0")
        self.lbl_loss.setStyleSheet(f"color: {COLOR_WARNING}; font-weight: bold; font-size: 15px;")
        left_panel.addWidget(self.lbl_loss)
        
        left_panel.addWidget(self._create_separator())

        left_panel.addWidget(self._create_heading("🎮 Controller View"))
        self.visualizer = ControllerVisualizer()
        left_panel.addWidget(self.visualizer, 0, Qt.AlignmentFlag.AlignCenter)

        self.lbl_log = QLabel("Waiting for input...")
        self.lbl_log.setFixedHeight(50) 
        self.lbl_log.setStyleSheet(f"""
            background-color: {COLOR_SURFACE}; 
            color: {COLOR_ACCENT};
            padding: 8px; 
            border-radius: 4px; 
            font-size: 12px; 
            font-family: 'Consolas', monospace;
            border: 1px solid {COLOR_LIGHT};
        """)
        self.lbl_log.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_log.setWordWrap(True)
        left_panel.addWidget(self.lbl_log)
        
        left_panel.addStretch(1)
        main_layout.addLayout(left_panel, 0) 
        
        vert_line = QFrame()
        vert_line.setFrameShape(QFrame.Shape.VLine)
        vert_line.setStyleSheet(f"color: {COLOR_LIGHT}; background-color: {COLOR_LIGHT};")
        main_layout.addWidget(vert_line)

        # RIGHT PANEL
        right_panel = QVBoxLayout()
        right_panel.setSpacing(20)

        right_panel.addWidget(self._create_heading("📈 Performance Metrics"))
        self.g_lat = CompactGraph("Latency", "ms", 100, "#E67E22")
        self.g_stb = CompactGraph("Stability", "%", 100, "#2ECC71")
        self.g_rate = CompactGraph("Rate", "Hz", 20, "#3498DB")
        
        right_panel.addWidget(self.g_lat)
        right_panel.addWidget(self.g_stb)
        right_panel.addWidget(self.g_rate)
        right_panel.addWidget(self._create_separator())

        # ROBOT FEED
        right_panel.addWidget(self._create_heading("🤖 Robot Feed"))
        
        # --- NEW: DEFAULT OFF ---
        self.btn_cam_toggle = QPushButton("Camera Feed: OFF")
        self.btn_cam_toggle.setCheckable(True)
        self.btn_cam_toggle.setChecked(False) # Initial State: Off
        self.btn_cam_toggle.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_cam_toggle.clicked.connect(self.toggle_camera_state)
        # Initial Style: OFF
        self.btn_cam_toggle.setStyleSheet(f"background-color: {COLOR_LIGHT}; color: #999; font-weight: bold; padding: 8px; border-radius: 4px;")
        right_panel.addWidget(self.btn_cam_toggle)

        self.robot_display = QLabel("Transmission Paused")
        self.robot_display.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.robot_display.setMinimumSize(400, 300) 
        self.robot_display.setStyleSheet(f"""
            background-color: {COLOR_TOUCHPAD}; 
            border: 2px solid {COLOR_LIGHT}; 
            border-radius: 10px;
            font-size: 14px; 
            color: #666;
            font-weight: bold;
        """)
        self.robot_display.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        
        right_panel.addWidget(self.robot_display)
        
        main_layout.addLayout(right_panel, 1)

        # Logic Setup
        self.thread = ControllerThread(path, profile)
        self.thread.inputEvent.connect(self.on_input)
        self.thread.stateUpdated.connect(self.on_state)
        
        self.ros = RosWorker()
        self.thread.stateDictUpdated.connect(self.ros.send_controller_state)
        self.ros.stats_signal.connect(self.update_stats)
        self.ros.video_signal.connect(self.update_video_feed)
        
        self.thread.start()
        self.ros.start()
    
    def _create_heading(self, text):
        lbl = QLabel(f"{text}")
        lbl.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {COLOR_TEXT_LIT}; margin-bottom: 5px;")
        return lbl

    def _create_separator(self):
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        line.setStyleSheet(f"background-color: {COLOR_LIGHT}; max-height: 1px; margin-top: 10px; margin-bottom: 10px;")
        return line

    def toggle_camera_state(self):
        is_on = self.btn_cam_toggle.isChecked()
        
        if is_on:
            self.btn_cam_toggle.setText("Camera Feed: ON")
            self.btn_cam_toggle.setStyleSheet(f"background-color: {COLOR_SUCCESS}; color: white; font-weight: bold; padding: 8px; border-radius: 4px;")
            self.robot_display.setText("Waiting for signal...")
        else:
            self.btn_cam_toggle.setText("Camera Feed: OFF")
            self.btn_cam_toggle.setStyleSheet(f"background-color: {COLOR_LIGHT}; color: #999; font-weight: bold; padding: 8px; border-radius: 4px;")
            
            # === FIX: Clear image first, then set text ===
            self.robot_display.clear() # Clears the Pixmap
            self.robot_display.setText("Transmission Paused") # Sets the Text
            self.robot_display.setAlignment(Qt.AlignmentFlag.AlignCenter) # Re-ensure alignment
        
        if self.ros.node:
            self.ros.node.camera_enabled = is_on

    def on_input(self, key, val):
        self.visualizer.set_input(key, val)
        
    def on_state(self, s): 
        self.lbl_log.setText(s)
        
    def update_stats(self, lat, stbl, loss, rate, connected):
        self.g_lat.add(lat)
        self.g_stb.add(stbl)
        self.g_rate.add(rate)
        
        self.lbl_loss.setText(f"Packet Loss: {loss}")
        
        if connected:
            self.lbl_conn.setText("● CONNECTED")
            self.lbl_conn.setStyleSheet(f"color: {COLOR_SUCCESS}; font-weight: bold; font-size: 16px; margin-bottom: 5px;")
        else:
            self.lbl_conn.setText("● DISCONNECTED")
            self.lbl_conn.setStyleSheet(f"color: {COLOR_WARNING}; font-weight: bold; font-size: 16px; margin-bottom: 5px;")

    def update_video_feed(self, q_img):
        pixmap = QPixmap.fromImage(q_img)
        w = self.robot_display.width()
        h = self.robot_display.height()
        scaled_pixmap = pixmap.scaled(w, h, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
        self.robot_display.setPixmap(scaled_pixmap)

    def closeEvent(self, e): 
        self.thread.stop(); self.ros.stop(); super().closeEvent(e)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    sel = DeviceSelectionDialog()
    if sel.exec() == QDialog.DialogCode.Accepted:
        win = ControllerGUI(sel.selected_path, sel.selected_profile)
        win.show() 
        sys.exit(app.exec())