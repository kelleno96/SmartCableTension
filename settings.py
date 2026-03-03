"""
Settings persistence module.
Saves and loads GUI/detector/workout settings to a JSON file.
Auto-saves on every change, auto-loads on startup.
"""

import json
import os

SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "settings.json")

# Default values — used when no settings file exists
DEFAULTS = {
    "hsv_h_low": 84,
    "hsv_h_high": 129,
    "hsv_s_low": 138,
    "hsv_s_high": 255,
    "hsv_v_low": 64,
    "hsv_v_high": 213,
    "center_x": 160,
    "center_y": 111,
    "mask_radius": 100,
    "power_level": 5.0,
    "target_speed": 90.0,
    "max_offset": 50,
    "pi_kp": 0.15,
    "pi_ki": 0.03,
    "velocity_alpha": 0.3,
}


def load_settings():
    """Load settings from disk, falling back to defaults for missing keys."""
    settings = dict(DEFAULTS)
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                saved = json.load(f)
            settings.update(saved)
        except (json.JSONDecodeError, IOError) as e:
            print(f"Warning: could not load settings ({e}), using defaults")
    return settings


def save_settings(settings):
    """Save settings to disk."""
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(settings, f, indent=2)
    except IOError as e:
        print(f"Warning: could not save settings ({e})")
