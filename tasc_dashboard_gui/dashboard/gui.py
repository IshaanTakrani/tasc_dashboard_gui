# dashboard/gui.py
import os
import json
from collections import deque
from evdev import InputDevice, ecodes, list_devices
from PyQt6.QtCore import Qt, QByteArray, QRectF, QRect
from PyQt6.QtGui import QPainter, QColor, QPen, QPainterPath, QPixmap
from PyQt6.QtWidgets import (QWidget, QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
                             QListWidget, QPushButton, QProgressBar, QFrame, QSizePolicy)
from PyQt6.QtSvg import QSvgRenderer

from config import *
from inputs import MappingWizard, ControllerThread
from ros_interface import RosWorker

class DeviceSelectionDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Select Controller"); self.setFixedSize(450, 400)
        self.setStyleSheet(f"background-color: {COLOR_BG}; color: {COLOR_TEXT_LIT}; font-family: 'Segoe UI';")
        self.selected_path = None; self.selected_profile = None

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("1. Select Hardware Device:"))
        self.list_widget = QListWidget()
        self.list_widget.setStyleSheet(f"QListWidget {{ background-color: {COLOR_SURFACE}; border: 1px solid {COLOR_LIGHT}; }} QListWidget::item:selected {{ background-color: {COLOR_ACCENT}; }}")
        self.list_widget.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        layout.addWidget(self.list_widget)

        layout.addWidget(QLabel("2. Controller Mapping:"))
        btn_layout = QHBoxLayout()
        self.btn_map = QPushButton("Create New Mapping"); self.btn_map.setEnabled(False)
        self.btn_map.setStyleSheet(f"background-color: {COLOR_LIGHT}; padding: 5px;")
        self.btn_map.clicked.connect(self.open_mapper)
        self.lbl_status = QLabel("No Selection"); self.lbl_status.setStyleSheet("color:#777")
        btn_layout.addWidget(self.btn_map); btn_layout.addWidget(self.lbl_status)
        layout.addLayout(btn_layout)

        self.connect_btn = QPushButton("Connect"); self.connect_btn.setEnabled(False)
        self.connect_btn.setStyleSheet(f"background-color: {COLOR_ACCENT}; padding:10px; font-weight:bold; border:none")
        self.connect_btn.clicked.connect(self.accept_selection)
        layout.addWidget(self.connect_btn)

        self.device_map = {}; self.refresh_devices()
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
        self.list_widget.clearSelection(); self.list_widget.setCurrentRow(-1)

    def on_selection_change(self):
        self.list_widget.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        if self.list_widget.currentRow() in self.device_map:
            self.btn_map.setEnabled(True); self.btn_map.setStyleSheet(f"background-color: {COLOR_ACCENT}; color: white; padding: 5px;")
            self.check_profile()
        else:
            self.btn_map.setEnabled(False); self.btn_map.setStyleSheet(f"background-color: {COLOR_LIGHT}; padding: 5px;")

    def check_profile(self):
        row = self.list_widget.currentRow(); dev = self.device_map[row]
        if os.path.exists(MAPPING_FILE):
            with open(MAPPING_FILE, 'r') as f:
                data = json.load(f)
                if dev.name in data:
                    self.selected_profile = data[dev.name]
                    self.lbl_status.setText("Profile Found ✓"); self.lbl_status.setStyleSheet(f"color:{COLOR_SUCCESS}")
                    self.connect_btn.setEnabled(True); return
        self.lbl_status.setText("Profile Missing"); self.lbl_status.setStyleSheet(f"color:{COLOR_WARNING}")
        self.connect_btn.setEnabled(False)

    def open_mapper(self):
        row = self.list_widget.currentRow(); dev = self.device_map[row]
        wiz = MappingWizard(dev.path, dev.name)
        if wiz.exec() == QDialog.DialogCode.Accepted:
            full = {}
            if os.path.exists(MAPPING_FILE):
                with open(MAPPING_FILE,'r') as f: full = json.load(f)
            full[dev.name] = wiz.mapping_result["map"]
            with open(MAPPING_FILE,'w') as f: json.dump(full, f, indent=4)
            self.check_profile()

    def accept_selection(self):
        self.selected_path = self.device_map[self.list_widget.currentRow()].path
        self.accept()

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

class ControllerVisualizer(QWidget):
    def __init__(self):
        super().__init__(); self.setFixedSize(320, 220) 
        self.col_off = "#222222"; self.col_on = "#5271FF"; self.col_body = "#444444"; self.col_trig_bg = "#333333"
        self.max_trig_w = 35.0; self.trig_l_w = 0.001; self.trig_r_w = 0.001
        self.colors = {k: self.col_off for k in ["BTN_SOUTH","BTN_EAST","BTN_WEST","BTN_NORTH","BTN_L1","BTN_R1","BTN_L3","BTN_R3","DPAD_UP","DPAD_DOWN","DPAD_LEFT","DPAD_RIGHT"]}
        self.joy_l = (0, 0); self.joy_r = (0, 0)
        self.reload_svg()

    def get_svg_template(self):
        return f"""
        <svg viewBox="-10 820 280 190" xmlns="http://www.w3.org/2000/svg">
            <path id="shell" d="m54.241 842.41c-0.47597-0.007-0.95982 0.005-1.438 0.0167-3.8255 0.097-7.7038 0.77663-11.333 1.6092-3.3992 0.77986-7.0506 2.0811-9.3643 4.6907-3.6256 4.0892-5.5124 15.425-5.5124 15.425s-10.619 7.0994-14.329 12.138c-3.4896 4.7395-5.3216 10.576-6.8991 16.246-1.0445 3.7543-1.291 7.6918-1.6435 11.573-2.3278 25.63-5.0152 48.958-3.0301 77.157 0.70842 10.064 5.0934 14.999 11.025 19.567 5.4892 4.2276 13.256 6.215 20.115 5.2385 5.3841-0.7665 10.178-4.4895 14.004-8.3542 7.9784-8.0589 15.483-28.016 17.137-29.394 1.6533-1.3777 3.8519-4.6907 3.8519-4.6907s5.2928 4.7838 8.5426 6.0603c4.5704 1.7951 9.7365 2.5903 14.603 1.9345 6.323-0.85208 12.786-3.2986 17.633-7.4469 3.447-2.95 7.173-11.573 7.173-11.573h26.449s3.726 8.6227 7.173 11.573c4.8474 4.1484 11.31 6.5948 17.633 7.4469 4.8663 0.65579 10.032-0.13934 14.603-1.9345 3.2498-1.2764 8.5426-6.0603 8.5426-6.0603s2.1986 3.313 3.8518 4.6907c1.6533 1.3778 9.141 21.335 17.119 29.394 3.8262 3.8648 8.6367 7.5878 14.021 8.3542 6.8593 0.9765 14.626-1.0109 20.115-5.2385 5.9315-4.5682 10.316-9.5038 11.025-19.567 1.985-28.199-0.70235-51.527-3.0301-77.157-0.35248-3.8809-0.61614-7.8184-1.6606-11.573-1.5775-5.6702-3.3924-11.507-6.882-16.246-3.7096-5.0382-14.329-12.138-14.329-12.138s-1.9041-11.335-5.5296-15.425c-2.3137-2.6096-5.9479-3.9109-9.3472-4.6907-7.2584-1.6652-15.525-2.7085-22.341 0.29103-3.9204 1.7252-6.595 6.0577-8.7994 9.3643s-3.304 13.781-3.304 13.781l-38.313-0.15416v-0.11974l-13.764 0.0514-13.781-0.0514v0.11974l-38.313 0.15416s-1.0996-10.475-3.304-13.781c-2.2044-3.3066-4.879-7.6391-8.7994-9.3643-2.9821-1.3123-6.2379-1.8667-9.5698-1.9174zm155.5 27.511zm-171.31 5.307zm148.55 14.62zm45.538 0zm-208.86 0.68478zm33.417 0zm88.705 4.023 14.687 4.8106-14.687 4.8277zm-51.345 0.33876h14.633v8.9607h-14.6zm-48.453 7.7245zm163.76 7.17zm-123.66 16.4c10.12 0 18.318 8.1976 18.318 18.318s-8.1976 18.335-18.318 18.335-18.318-8.2148-18.318-18.335 8.1977-18.318 18.318-18.318zm82.721 0c10.12 0 18.318 8.1976 18.318 18.318s-8.1976 18.335-18.318 18.335-18.318-8.2148-18.318-18.335 8.1976-18.318 18.318-18.318z" fill="{self.col_body}" stroke="#222" stroke-width="2" fill-rule="evenodd" />
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
        self.renderer = QSvgRenderer(QByteArray(svg_content.encode('utf-8'))); self.update()

    def set_input(self, key, val):
        if key == "L2_INPUT": self.trig_l_w = max(0.001, val * self.max_trig_w)
        elif key == "R2_INPUT": self.trig_r_w = max(0.001, val * self.max_trig_w)
        elif key in self.colors: self.colors[key] = self.col_on if val > 0.4 else self.col_off
        elif key == "L_STICK_X": self.joy_l = (val * 8.0, self.joy_l[1])
        elif key == "L_STICK_Y": self.joy_l = (self.joy_l[0], val * 8.0)
        elif key == "R_STICK_X": self.joy_r = (val * 8.0, self.joy_r[1])
        elif key == "R_STICK_Y": self.joy_r = (self.joy_r[0], val * 8.0)
        self.reload_svg()

    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        if self.renderer.isValid(): self.renderer.render(p, QRectF(self.rect()))
        else: p.setBrush(QColor("red")); p.drawRect(self.rect())

class RobotArmWidget(QWidget):
    def __init__(self):
        super().__init__(); self.setFixedSize(150, 300); self.angle = 90.0 
        self.setStyleSheet(f"background-color: {COLOR_TOUCHPAD}; border: 2px solid {COLOR_LIGHT}; border-radius: 10px;")

    def set_angle(self, angle):
        self.angle = angle; self.update()

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w = self.width(); h = self.height()
        p.fillRect(10, h - 20, w - 20, 10, QColor(COLOR_LIGHT))
        body_w = 110; body_h = 40; body_x = (w - body_w) / 2; body_y = h - 20 - body_h
        p.setBrush(QColor(COLOR_ACCENT)); p.setPen(Qt.PenStyle.NoPen)
        p.drawRoundedRect(int(body_x), int(body_y), body_w, body_h, 5, 5)
        pivot_x = body_x + body_w - 15; pivot_y = body_y + 10 
        p.save(); p.translate(pivot_x, pivot_y); p.rotate(-self.angle) 
        arm_w = 80; arm_h = 12
        p.setBrush(QColor(COLOR_WARNING)); p.drawRoundedRect(0, -arm_h//2, arm_w, arm_h, 4, 4)
        p.setBrush(QColor("#FFF")); p.drawEllipse(arm_w - 5, -8, 16, 16); p.restore() 
        p.setBrush(QColor(COLOR_BG)); p.drawEllipse(int(pivot_x - 4), int(pivot_y - 4), 8, 8)
        p.setPen(QColor("#FFF")); p.drawText(QRect(0, 10, w, 20), Qt.AlignmentFlag.AlignCenter, f"Angle: {self.angle:.1f}°")

class ControllerGUI(QWidget):
    def __init__(self, path, profile):
        super().__init__()
        self.setWindowTitle("ROS2 Controller Dashboard"); self.resize(1150, 750)
        self.setStyleSheet(f"background:{COLOR_BG}; font-family:'Segoe UI'; color:{COLOR_TEXT_LIT};")
        
        main_layout = QHBoxLayout(self); left_panel = QVBoxLayout(); right_panel = QVBoxLayout()
        
        # Left Panel
        self._create_heading("📡 Connection Status", left_panel)
        self.lbl_conn = QLabel("● DISCONNECTED"); self.lbl_conn.setStyleSheet(f"color: {COLOR_WARNING}; font-weight: bold; font-size: 16px;")
        left_panel.addWidget(self.lbl_conn)
        self.lbl_loss = QLabel("Packet Loss: 0"); self.lbl_loss.setStyleSheet(f"color: {COLOR_WARNING}; font-weight: bold; font-size: 15px;")
        left_panel.addWidget(self.lbl_loss)
        self._create_separator(left_panel)
        self._create_heading("🎮 Controller View", left_panel)
        self.visualizer = ControllerVisualizer(); left_panel.addWidget(self.visualizer, 0, Qt.AlignmentFlag.AlignCenter)
        self.lbl_log = QLabel("Waiting for input..."); self.lbl_log.setFixedHeight(50)
        self.lbl_log.setStyleSheet(f"background-color: {COLOR_SURFACE}; color: {COLOR_ACCENT}; padding: 8px; border-radius: 4px; font-family: 'Consolas'; border: 1px solid {COLOR_LIGHT};")
        self.lbl_log.setAlignment(Qt.AlignmentFlag.AlignCenter); self.lbl_log.setWordWrap(True); left_panel.addWidget(self.lbl_log)
        left_panel.addStretch(1); main_layout.addLayout(left_panel, 0) 
        
        line = QFrame(); line.setFrameShape(QFrame.Shape.VLine); line.setStyleSheet(f"background-color: {COLOR_LIGHT};"); main_layout.addWidget(line)

        # Right Panel
        self._create_heading("📈 Performance Metrics", right_panel)
        stats_layout = QHBoxLayout(); stats_layout.setSpacing(10)
        self.g_lat = CompactGraph("Latency", "ms", 100, "#E67E22"); stats_layout.addWidget(self.g_lat)
        self.g_stb = CompactGraph("Stability", "%", 100, "#2ECC71"); stats_layout.addWidget(self.g_stb)
        self.g_rate = CompactGraph("Rate", "Hz", 20, "#3498DB"); stats_layout.addWidget(self.g_rate)
        right_panel.addLayout(stats_layout); self._create_separator(right_panel)

        self._create_heading("🤖 Robot Feed & State", right_panel)
        self.btn_cam_toggle = QPushButton("Camera Feed: OFF"); self.btn_cam_toggle.setCheckable(True); self.btn_cam_toggle.clicked.connect(self.toggle_camera_state)
        self.btn_cam_toggle.setStyleSheet(f"background-color: {COLOR_LIGHT}; color: #999; font-weight: bold; padding: 8px; border-radius: 4px;")
        right_panel.addWidget(self.btn_cam_toggle)

        feed_layout = QHBoxLayout(); feed_layout.setSpacing(15)
        self.robot_display = QLabel("Transmission Paused"); self.robot_display.setMinimumSize(400, 300)
        self.robot_display.setAlignment(Qt.AlignmentFlag.AlignCenter); self.robot_display.setStyleSheet(f"background-color: {COLOR_TOUCHPAD}; border: 2px solid {COLOR_LIGHT}; border-radius: 10px; font-size: 14px; color: #666; font-weight: bold;")
        self.robot_display.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        feed_layout.addWidget(self.robot_display, 1)
        self.arm_display = RobotArmWidget(); feed_layout.addWidget(self.arm_display, 0)
        right_panel.addLayout(feed_layout); main_layout.addLayout(right_panel, 1)

        self.thread = ControllerThread(path, profile); self.thread.inputEvent.connect(self.visualizer.set_input); self.thread.stateUpdated.connect(self.lbl_log.setText)
        self.ros = RosWorker(); self.thread.stateDictUpdated.connect(self.ros.send_controller_state); self.ros.stats_signal.connect(self.update_stats); self.ros.video_signal.connect(self.update_video_feed)
        self.ros.telemetry_signal.connect(self.arm_display.set_angle)
        self.thread.start(); self.ros.start()

    def _create_heading(self, text, layout):
        lbl = QLabel(f"{text}"); lbl.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {COLOR_TEXT_LIT}; margin-bottom: 5px;")
        layout.addWidget(lbl)

    def _create_separator(self, layout):
        line = QFrame(); line.setFrameShape(QFrame.Shape.HLine); line.setFrameShadow(QFrame.Shadow.Sunken)
        line.setStyleSheet(f"background-color: {COLOR_LIGHT}; max-height: 1px; margin-top: 10px; margin-bottom: 10px;")
        layout.addWidget(line)

    def toggle_camera_state(self):
        if self.btn_cam_toggle.isChecked():
            self.btn_cam_toggle.setText("Camera Feed: ON"); self.btn_cam_toggle.setStyleSheet(f"background-color: {COLOR_SUCCESS}; color: white; font-weight: bold; padding: 8px; border-radius: 4px;")
            self.robot_display.setText("Waiting for signal...")
        else:
            self.btn_cam_toggle.setText("Camera Feed: OFF"); self.btn_cam_toggle.setStyleSheet(f"background-color: {COLOR_LIGHT}; color: #999; font-weight: bold; padding: 8px; border-radius: 4px;")
            self.robot_display.clear(); self.robot_display.setText("Transmission Paused"); self.robot_display.setAlignment(Qt.AlignmentFlag.AlignCenter)
        if self.ros.node: self.ros.node.camera_enabled = self.btn_cam_toggle.isChecked()

    def update_stats(self, lat, stbl, loss, rate, connected):
        self.g_lat.add(lat); self.g_stb.add(stbl); self.g_rate.add(rate); self.lbl_loss.setText(f"Packet Loss: {loss}")
        self.lbl_conn.setText("● CONNECTED" if connected else "● DISCONNECTED")
        self.lbl_conn.setStyleSheet(f"color: {COLOR_SUCCESS if connected else COLOR_WARNING}; font-weight: bold; font-size: 16px;")

    def update_video_feed(self, q_img):
        self.robot_display.setPixmap(QPixmap.fromImage(q_img).scaled(self.robot_display.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))

    def closeEvent(self, e): self.thread.stop(); self.ros.stop(); super().closeEvent(e)