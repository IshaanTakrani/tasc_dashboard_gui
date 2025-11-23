from evdev import InputDevice, categorize, ecodes

# Replace this with your controller's event file, e.g.:
#   /dev/input/event3
device_path = '/dev/input/event6'  # change X to match your controller

gamepad = InputDevice(device_path)

print("Listening for controller input...")
print("Device:", gamepad)

for event in gamepad.read_loop():
    # Only care about key presses and analog movement
    if event.type == ecodes.EV_KEY:
        key_event = categorize(event)
        print(f"BUTTON: {key_event.keycode}, VALUE: {event.value}")

    elif event.type == ecodes.EV_ABS:
        abs_event = categorize(event)
        code = ecodes.ABS[event.code] if event.code in ecodes.ABS else event.code
        print(f"ANALOG: {code}, VALUE: {event.value}")

    elif event.type == ecodes.EV_SYN:
        # Synchronization events (can ignore or log)
        pass
