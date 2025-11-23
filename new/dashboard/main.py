#!/usr/bin/env python3
"""
Dashboard Entry Point.
Initializes the Application, runs the Device Selection wizard, 
and launches the Main Controller GUI.
"""

import sys
from PyQt6.QtWidgets import QApplication, QDialog

# Local Imports
from inputs import DeviceSelectionDialog
from gui import ControllerGUI

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 1. Launch Device Selection Dialog
    sel = DeviceSelectionDialog()
    
    if sel.exec() == QDialog.DialogCode.Accepted:
        # 2. If valid selection, launch Main Dashboard
        win = ControllerGUI(sel.selected_path, sel.selected_profile)
        win.show() 
        sys.exit(app.exec())