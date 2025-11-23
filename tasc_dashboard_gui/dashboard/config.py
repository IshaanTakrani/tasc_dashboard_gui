# dashboard/config.py

MAPPING_FILE = "controller_mappings.json"
STICK_DEADZONE = 0.10
AXIS_DETECT_THRESHOLD = 0.5

# Colors
COLOR_BG        = "#1A1A1D"
COLOR_SURFACE   = "#2C2C31"
COLOR_TOUCHPAD  = "#151518"
COLOR_LIGHT     = "#484850"
COLOR_ACCENT    = "#5271FF"
COLOR_SUCCESS   = "#2ECC71"
COLOR_WARNING   = "#F1C40F"
COLOR_TEXT_LIT  = "#FFFFFF"

# Mapping Template
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
    ("BTN_R3",    "Press RIGHT Stick Down (R3)", "btn_discrete")
]