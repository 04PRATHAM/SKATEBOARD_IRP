"""
WASD manual controller for testing phase only (isolated from core project).

Maps:
  W = forward  (accelerate +0.4 m/s per press, max 1.5)
  S = backward (decelerate -0.4, max -1.0)
  A = left     (steer +0.15 rad per press, max 0.6)
  D = right    (steer -0.15 rad per press, max 0.6)
  Space = brake (speed 0, steer 0)
  R = reset steer to 0
  Q/Esc = quit (handled by window close)

This is a *testing* module — not part of the production pipeline.
Core project remains under isaac_sim/scripts/{vehicle,sensors,utils,bridge}.
"""
import math

class WASDState:
    def __init__(self, max_speed=1.5, max_steer=0.6):
        self.speed = 0.0
        self.steer = 0.0
        self.max_speed = float(max_speed)
        self.min_speed = -1.0
        self.max_steer = float(max_steer)
        self.speed_step = 0.4
        self.steer_step = 0.15

    def handle_key(self, key: str):
        k = key.lower()
        if k == "w":
            self.speed = min(self.max_speed, self.speed + self.speed_step)
        elif k == "s":
            self.speed = max(self.min_speed, self.speed - self.speed_step)
        elif k == "a":
            self.steer = min(self.max_steer, self.steer + self.steer_step)
        elif k == "d":
            self.steer = max(-self.max_steer, self.steer - self.steer_step)
        elif k == " " or k == "space":
            self.speed = 0.0
            self.steer = 0.0
        elif k == "r":
            self.steer = 0.0
        # Clamp
        self.speed = max(self.min_speed, min(self.max_speed, self.speed))
        self.steer = max(-self.max_steer, min(self.max_steer, self.steer))

    def __repr__(self):
        return f"speed={self.speed:+.2f} m/s steer={self.steer:+.2f} rad"

def subscribe_wasd(wasd_state):
    """Subscribe to Isaac Sim keyboard events and drive a WASDState.

    Uses carb.input keyboard-event subscription on the default app window's
    keyboard (the correct approach in Isaac Sim 5.1 — carb.input has no
    ``poll``/``get_key_state``; it is event-driven). The viewport must have
    focus for keys to register.

    Returns (input_interface, keyboard, subscription_id). Keep a reference to
    all three for the lifetime of the sim, and call
    ``input_interface.unsubscribe_to_keyboard_events(keyboard, sub_id)`` on exit.
    """
    import carb.input
    import omni.appwindow

    app_window = omni.appwindow.get_default_app_window()
    keyboard = app_window.get_keyboard()
    input_iface = carb.input.acquire_input_interface()

    # KeyboardInput.name -> single char understood by WASDState.handle_key
    key_map = {
        "W": "w", "S": "s", "A": "a", "D": "d",
        "SPACE": " ", "R": "r",
        "UP": "w", "DOWN": "s", "LEFT": "a", "RIGHT": "d",
    }
    press_types = (carb.input.KeyboardEventType.KEY_PRESS,
                   carb.input.KeyboardEventType.KEY_REPEAT)

    def _on_kbd(event, *args):
        try:
            if event.type in press_types:
                name = event.input.name
                if name in key_map:
                    wasd_state.handle_key(key_map[name])
        except Exception:
            pass
        return True

    sub_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_kbd)
    return input_iface, keyboard, sub_id


def poll_keyboard():
    """Deprecated stub kept for backward compatibility.

    carb.input is event-driven in Isaac Sim 5.1 (no poll/get_key_state), so
    real input goes through :func:`subscribe_wasd`. This always returns an
    empty set.
    """
    return set()
