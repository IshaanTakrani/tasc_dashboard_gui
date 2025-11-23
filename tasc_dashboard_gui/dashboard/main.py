# dashboard/main.py
#!/usr/bin/env python3
import sys
import os
# Source ROS2 environment automatically
os.system('bash -c "source /opt/ros/humble/setup.bash && echo ROS2 Environment Sourced"')

from PyQt6.QtWidgets import QApplication, QDialog
from gui import DeviceSelectionDialog, ControllerGUI

if __name__ == "__main__":
    app = QApplication(sys.argv)
    sel = DeviceSelectionDialog()
    if sel.exec() == QDialog.DialogCode.Accepted:
        win = ControllerGUI(sel.selected_path, sel.selected_profile)
        win.show() 
        sys.exit(app.exec())