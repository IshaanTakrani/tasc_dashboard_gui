# dashboard/inputs.py
import time
from evdev import InputDevice, ecodes
from PyQt6.QtCore import QThread, pyqtSignal, QTimer, Qt
from PyQt6.QtWidgets import QDialog, QVBoxLayout, QLabel, QProgressBar, QListWidget, QPushButton
from config import *

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
        rng = info.max - info.min
        if rng == 0: rng = 1
        return 2 * (val - info.min) / rng - 1.0

    def run(self):
        try:
            dev = InputDevice(self.device_path)
            
            # PHASE 1: Init
            for code in dev.capabilities().get(ecodes.EV_KEY, []):
                self.current_state[code] = {"type": "btn", "value": 0.0}
                self.baseline_state[code] = 0.0 

            for code in dev.capabilities().get(ecodes.EV_ABS, []):
                try:
                    info = dev.absinfo(code)
                    val = self._normalize(info.value, info)
                    if code in [ecodes.ABS_Z, ecodes.ABS_RZ]:
                        if val == 0.0 and info.min < 0: val = -1.0
                    self.current_state[code] = {"type": "axis", "value": val}
                    self.baseline_state[code] = val 
                except: pass

            # PHASE 2: Calibration
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

            # PHASE 3: Loop
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

                packet = {"current": self.current_state, "baseline": self.baseline_state}
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
        self.stability_counter = 0
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
        _, txt, _ = LOGICAL_MAP_TEMPLATE[self.current_step]
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
        is_currently_silent = True
        offending_input = ""
        
        for code, data in full_state.items():
            if data["type"] == "btn":
                if data["value"] == 1: 
                    is_currently_silent = False; offending_input = f"Btn {code}"; break
            elif data["type"] == "axis":
                if abs(data["value"] - baseline.get(code, 0.0)) > 0.35:
                    is_currently_silent = False; offending_input = f"Axis {code}"; break
        
        if self.wait_for_neutral:
            if is_currently_silent: self.stability_counter += 1
            else: self.stability_counter = 0; self.lbl_debug.setText(f"Wait for Release: {offending_input}")
            if self.stability_counter > self.REQUIRED_STABLE_FRAMES:
                self.wait_for_neutral = False
                self.lbl_debug.setText("Status: Listening")
                _, txt, _ = LOGICAL_MAP_TEMPLATE[self.current_step]
                self.lbl_instruction.setText(txt)
                self.lbl_instruction.setStyleSheet(f"font-size: 20px; color: {COLOR_ACCENT}; font-weight: bold; margin: 20px;")
            return 

        best_code = None; best_data = None; max_strength = 0
        for code, data in full_state.items():
            strength = data["value"] if data["type"] == "btn" else abs(data["value"] - baseline.get(code, 0.0))
            if strength > max_strength: max_strength = strength; best_code = code; best_data = data

        if best_code is not None:
            target_log, _, target_type = LOGICAL_MAP_TEMPLATE[self.current_step]
            valid = False; map_entry = {}
            ACTIVATE_THRESH = 0.6

            if target_type == "axis_full" and best_data["type"] == "axis" and max_strength > ACTIVATE_THRESH:
                map_entry = {"logical": target_log, "type": "axis"}; valid = True
            elif target_type == "btn_discrete":
                if best_data["type"] == "btn" and best_data["value"] == 1:
                    map_entry = {"logical": target_log, "type": "btn"}; valid = True
                elif best_data["type"] == "axis" and max_strength > 0.8:
                    direction = 1 if (best_data["value"] - baseline.get(best_code, 0.0)) > 0 else -1
                    map_entry = {"logical": target_log, "type": "axis_as_btn", "trigger_val": direction}; valid = True
            elif target_type == "any":
                if best_data["type"] == "btn" and best_data["value"] == 1:
                    map_entry = {"logical": target_log, "type": "btn"}; valid = True
                elif best_data["type"] == "axis" and max_strength > ACTIVATE_THRESH:
                    map_entry = {"logical": target_log, "type": "axis"}; valid = True

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
                    except: axis_calib[code] = {"min": 0, "max": 255}

            active_states = {} 
            for event in gamepad.read_loop():
                if not self.running: break
                code_str = str(event.code)
                if code_str not in self.profile: continue
                
                mapping = self.profile[code_str]
                logical = mapping["logical"]; m_type = mapping["type"]
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
                        if logical in ["L2_INPUT", "R2_INPUT"]:
                            norm = (norm + 1.0) / 2.0
                            norm = max(0.0, min(1.0, norm))
                        if abs(norm) < STICK_DEADZONE: norm = 0.0
                        self.inputEvent.emit(logical, norm)
                        if abs(norm) > 0: active_states[logical] = round(norm, 2)
                        elif logical in active_states: del active_states[logical]
                        has_changed = True
                    
                    elif m_type == "axis_as_btn":
                        pair_map = {
                            "DPAD_DOWN":  ("DPAD_UP", "DPAD_DOWN"),
                            "DPAD_UP":    ("DPAD_UP", "DPAD_DOWN"),
                            "DPAD_RIGHT": ("DPAD_LEFT", "DPAD_RIGHT"),
                            "DPAD_LEFT":  ("DPAD_LEFT", "DPAD_RIGHT")
                        }
                        if logical in pair_map:
                            neg_btn, pos_btn = pair_map[logical]
                            is_neg_pressed = (norm < -0.5); is_pos_pressed = (norm > 0.5)
                            val_neg = 1.0 if is_neg_pressed else 0.0
                            self.inputEvent.emit(neg_btn, val_neg)
                            if val_neg: active_states[neg_btn] = 1
                            elif neg_btn in active_states: del active_states[neg_btn]
                            val_pos = 1.0 if is_pos_pressed else 0.0
                            self.inputEvent.emit(pos_btn, val_pos)
                            if val_pos: active_states[pos_btn] = 1
                            elif pos_btn in active_states: del active_states[pos_btn]
                        else:
                            target = mapping["trigger_val"]
                            is_pressed = (target < 0 and norm < -0.5) or (target > 0 and norm > 0.5)
                            val = 1.0 if is_pressed else 0.0
                            self.inputEvent.emit(logical, val)
                            if val: active_states[logical] = 1
                            elif logical in active_states: del active_states[logical]
                        has_changed = True

                state_str = ", ".join([f"{k}:{v}" for k,v in active_states.items()])
                if not state_str: state_str = "Idle"
                self.stateUpdated.emit(state_str)
                if has_changed or len(active_states) == 0: self.stateDictUpdated.emit(active_states.copy())
        except Exception as e: print(f"Controller Error: {e}")

    def stop(self):
        self.running = False