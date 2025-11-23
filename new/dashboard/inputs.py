"""
Handles hardware input devices (evdev), controller mapping logic, 
and the device selection wizard.
"""

import os
import json
import time
from evdev import InputDevice, ecodes, list_devices
from PyQt6.QtCore import QThread, pyqtSignal, QTimer, Qt
from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QLabel, QProgressBar, 
    QListWidget, QPushButton, QHBoxLayout
)

# Import Configuration
import config

class MappingWorker(QThread):
    """
    Background thread for reading raw input events during the mapping calibration phase.
    It normalizes axis data and handles trigger quirks.
    """
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
        """Helper to safely normalize any axis to -1.0 ... 1.0."""
        rng = info.max - info.min
        if rng == 0: rng = 1
        return 2 * (val - info.min) / rng - 1.0

    def run(self):
        try:
            dev = InputDevice(self.device_path)
            
            # Phase 1: Initialize Buttons and Axes
            for code in dev.capabilities().get(ecodes.EV_KEY, []):
                self.current_state[code] = {"type": "btn", "value": 0.0}
                self.baseline_state[code] = 0.0 

            for code in dev.capabilities().get(ecodes.EV_ABS, []):
                try:
                    info = dev.absinfo(code)
                    val = self._normalize(info.value, info)
                    # Fix for uninitialized triggers (L2/R2 often rest at -1 or 0 depending on driver)
                    if code in [ecodes.ABS_Z, ecodes.ABS_RZ]:
                        if val == 0.0 and info.min < 0:
                            val = -1.0
                    self.current_state[code] = {"type": "axis", "value": val}
                    self.baseline_state[code] = val 
                except: 
                    pass

            # Phase 2: Calibration (Determine resting noise)
            samples = 25
            for i in range(samples):
                if not self.running: return
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

            # Phase 3: Runtime Loop
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
    """
    GUI Wizard that guides the user through pressing specific buttons 
    to create a logical mapping profile.
    """
    def __init__(self, device_path, device_name):
        super().__init__()
        self.setWindowTitle(f"Map Controller: {device_name}")
        self.resize(550, 450)
        self.setStyleSheet(f"background-color: {config.COLOR_BG}; color: {config.COLOR_TEXT_LIT}; font-family: 'Segoe UI';")
        
        self.current_step = 0
        self.mapping_result = {"device_name": device_name, "map": {}}
        self.stability_counter = 0
        self.REQUIRED_STABLE_FRAMES = 25 

        # UI Layout
        layout = QVBoxLayout(self)
        self.lbl_step = QLabel("Initializing...")
        self.lbl_step.setStyleSheet("color: #777; font-weight:bold;")
        layout.addWidget(self.lbl_step)

        self.lbl_instruction = QLabel("Don't touch the controller!\nCalibrating resting points...")
        self.lbl_instruction.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_instruction.setStyleSheet(f"font-size: 20px; color: {config.COLOR_WARNING}; font-weight: bold; margin: 20px;")
        layout.addWidget(self.lbl_instruction)
        
        self.lbl_debug = QLabel("")
        self.lbl_debug.setStyleSheet("color: #555; font-size: 10px; font-family: monospace;")
        self.lbl_debug.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.lbl_debug)

        self.pbar = QProgressBar()
        self.pbar.setRange(0, 100)
        self.pbar.setStyleSheet(f"QProgressBar::chunk {{ background-color: {config.COLOR_ACCENT}; }}")
        layout.addWidget(self.pbar)

        self.list_preview = QListWidget()
        self.list_preview.setStyleSheet(f"background-color: {config.COLOR_SURFACE}; border: none;")
        layout.addWidget(self.list_preview)

        self.btn_skip = QPushButton("Skip This Input")
        self.btn_skip.clicked.connect(self.skip_step)
        self.btn_skip.hide()

        self.btn_start = QPushButton("Start Mapping")
        self.btn_start.clicked.connect(self.start_mapping)
        self.btn_start.setStyleSheet(f"background-color: {config.COLOR_SUCCESS}; padding: 10px; font-weight: bold; color:white;")
        self.btn_start.setEnabled(False) 
        
        layout.addWidget(self.btn_skip)
        layout.addWidget(self.btn_start)

        # Logic Setup
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
            self.lbl_instruction.setStyleSheet(f"font-size: 20px; color: {config.COLOR_SUCCESS}; font-weight: bold; margin: 20px;")
            self.btn_start.setEnabled(True)
            self.pbar.setRange(0, len(config.LOGICAL_MAP_TEMPLATE))
            self.pbar.setValue(0)

    def start_mapping(self):
        self.listening = True
        self.wait_for_neutral = True 
        self.stability_counter = 0 
        self.btn_start.hide()
        self.btn_skip.show()
        self.update_ui()

    def update_ui(self):
        if self.current_step >= len(config.LOGICAL_MAP_TEMPLATE):
            self.finish_mapping()
            return
        
        log, txt, typ = config.LOGICAL_MAP_TEMPLATE[self.current_step]
        self.lbl_step.setText(f"Step {self.current_step + 1}/{len(config.LOGICAL_MAP_TEMPLATE)}")
        self.pbar.setValue(self.current_step)
        
        self.wait_for_neutral = True 
        self.stability_counter = 0 
        self.lbl_instruction.setStyleSheet(f"font-size: 20px; color: {config.COLOR_WARNING}; font-weight: bold; margin: 20px;")
        self.lbl_instruction.setText(f"{txt}\n(Release all buttons...)")

    def process_state_snapshot(self, packet):
        if not self.worker.is_calibrated or not self.listening: return
        if self.current_step >= len(config.LOGICAL_MAP_TEMPLATE): return
        
        full_state = packet["current"]
        baseline = packet["baseline"]

        # Check for Silence (No input)
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
                if abs(curr_val - base_val) > 0.35:
                    is_currently_silent = False
                    offending_input = f"Axis {code} ({curr_val:.2f})"
                    break
        
        # Stability Lock
        if self.wait_for_neutral:
            if is_currently_silent:
                self.stability_counter += 1
            else:
                self.stability_counter = 0 
                self.lbl_debug.setText(f"Wait for Release: {offending_input}")

            if self.stability_counter > self.REQUIRED_STABLE_FRAMES:
                self.wait_for_neutral = False
                self.lbl_debug.setText("Status: Listening")
                log, txt, typ = config.LOGICAL_MAP_TEMPLATE[self.current_step]
                self.lbl_instruction.setText(txt)
                self.lbl_instruction.setStyleSheet(f"font-size: 20px; color: {config.COLOR_ACCENT}; font-weight: bold; margin: 20px;")
            return 

        # Input Detection
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
            target_log, _, target_type = config.LOGICAL_MAP_TEMPLATE[self.current_step]
            valid = False
            map_entry = {}
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


class ControllerThread(QThread):
    """
    Runtime thread that connects to the specific device, applies the loaded mapping,
    and emits signals for buttons and axes.
    """
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
            
            # Pre-calc axis ranges
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
                    cal = axis_calib[int(code_str)]
                    rng = cal["max"] - cal["min"]
                    if rng == 0: rng = 1
                    norm = 2 * (event.value - cal["min"]) / rng - 1.0
                    
                    if m_type == "axis":
                        # Remap triggers from -1..1 to 0..1
                        if logical in ["L2_INPUT", "R2_INPUT"]:
                            norm = (norm + 1.0) / 2.0
                            norm = max(0.0, min(1.0, norm))

                        if abs(norm) < config.STICK_DEADZONE: norm = 0.0
                        
                        self.inputEvent.emit(logical, norm)
                        
                        if abs(norm) > 0: 
                            active_states[logical] = round(norm, 2)
                        elif logical in active_states: 
                            del active_states[logical]
                        has_changed = True
                    
                    elif m_type == "axis_as_btn":
                        # Handle D-Pad on Axes (Hat switch)
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
                            # Standard Axis-to-Button
                            target = mapping["trigger_val"]
                            is_pressed = False
                            if target < 0 and norm < -0.5: is_pressed = True
                            if target > 0 and norm > 0.5: is_pressed = True
                            val = 1.0 if is_pressed else 0.0
                            self.inputEvent.emit(logical, val)
                            if val: active_states[logical] = 1
                            elif logical in active_states: del active_states[logical]
                        has_changed = True

                state_str = ", ".join([f"{k}:{v}" for k,v in active_states.items()])
                if not state_str: state_str = "Idle"
                self.stateUpdated.emit(state_str)
                
                if has_changed or len(active_states) == 0:
                    self.stateDictUpdated.emit(active_states.copy())

        except Exception as e:
            print(f"Controller Error: {e}")

    def stop(self):
        self.running = False


class DeviceSelectionDialog(QDialog):
    """
    Initial Dialog to select the Input Device and Load/Create Mappings.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Select Controller")
        self.setFixedSize(450, 400)
        self.setStyleSheet(f"background-color: {config.COLOR_BG}; color: {config.COLOR_TEXT_LIT}; font-family: 'Segoe UI';")
        self.selected_path = None
        self.selected_profile = None

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("1. Select Hardware Device:"))
        self.list_widget = QListWidget()
        self.list_widget.setStyleSheet(f"""
            QListWidget {{ background-color: {config.COLOR_SURFACE}; border: 1px solid {config.COLOR_LIGHT}; }}
            QListWidget::item:selected {{ background-color: {config.COLOR_ACCENT}; }}
        """)
        self.list_widget.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        layout.addWidget(self.list_widget)

        layout.addWidget(QLabel("2. Controller Mapping:"))
        btn_layout = QHBoxLayout()
        
        self.btn_map = QPushButton("Create New Mapping")
        self.btn_map.setStyleSheet(f"background-color: {config.COLOR_LIGHT}; padding: 5px;")
        self.btn_map.setEnabled(False) 
        self.btn_map.clicked.connect(self.open_mapper)
        
        self.lbl_status = QLabel("No Selection"); self.lbl_status.setStyleSheet("color:#777")
        btn_layout.addWidget(self.btn_map); btn_layout.addWidget(self.lbl_status)
        layout.addLayout(btn_layout)

        self.connect_btn = QPushButton("Connect"); self.connect_btn.setEnabled(False)
        self.connect_btn.setStyleSheet(f"background-color: {config.COLOR_ACCENT}; padding:10px; font-weight:bold; border:none")
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
        self.list_widget.clearSelection()
        self.list_widget.setCurrentRow(-1)

    def on_selection_change(self):
        self.list_widget.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        row = self.list_widget.currentRow()
        if row in self.device_map:
            self.btn_map.setEnabled(True)
            self.btn_map.setStyleSheet(f"background-color: {config.COLOR_ACCENT}; color: white; padding: 5px;")
            self.check_profile()
        else:
            self.btn_map.setEnabled(False)
            self.btn_map.setStyleSheet(f"background-color: {config.COLOR_LIGHT}; padding: 5px;")

    def check_profile(self):
        row = self.list_widget.currentRow()
        if row not in self.device_map: return
        dev = self.device_map[row]
        
        if os.path.exists(config.MAPPING_FILE):
            try:
                with open(config.MAPPING_FILE, 'r') as f:
                    # Handle empty file check
                    content = f.read().strip()
                    if not content:
                        data = {}
                    else:
                        data = json.loads(content)
                        
                if dev.name in data:
                    self.selected_profile = data[dev.name]
                    self.lbl_status.setText("Profile Found ✓"); self.lbl_status.setStyleSheet(f"color:{config.COLOR_SUCCESS}")
                    self.connect_btn.setEnabled(True)
                    return
            except (json.JSONDecodeError, OSError):
                # If file is corrupted, treat as missing
                pass

        self.lbl_status.setText("Profile Missing"); self.lbl_status.setStyleSheet(f"color:{config.COLOR_WARNING}")
        self.connect_btn.setEnabled(False)

    def open_mapper(self):
        row = self.list_widget.currentRow()
        if row not in self.device_map: return
        dev = self.device_map[row]
        wiz = MappingWizard(dev.path, dev.name)
        if wiz.exec() == QDialog.DialogCode.Accepted:
            full = {}
            if os.path.exists(config.MAPPING_FILE):
                try:
                    with open(config.MAPPING_FILE,'r') as f:
                        content = f.read().strip()
                        if content:
                            full = json.loads(content)
                except (json.JSONDecodeError, OSError):
                    full = {} # Start fresh if corrupt

            full[dev.name] = wiz.mapping_result["map"]
            
            with open(config.MAPPING_FILE,'w') as f: 
                json.dump(full, f, indent=4)
            self.check_profile()

    def accept_selection(self):
        row = self.list_widget.currentRow()
        self.selected_path = self.device_map[row].path
        self.accept()