# Agents.md — Smart Cable Tension Project

## Project Overview
A smart cable tension workout system. A motor controlled via Arduino serial provides adjustable resistance while an external webcam tracks cable angle via an orange tape marker on a black disc centered on the motor axle. Closed-loop feedback adjusts motor power based on the selected workout mode.

## Hardware Setup
- **Motor controller**: Arduino on serial port `/dev/cu.usbserial-10` at 115200 baud
- **Protocol**: Send 2 bytes big-endian (uint16) 0–65535 → 0 = max negative speed, 32767 = stop, 65535 = max positive speed. Arduino maps to 1000–2000 µs ESC pulse.
- **External webcam**: OpenCV camera index **0**, running at 320×240 for speed
- **Marker**: Dull orange tape strip on a black circle centered on the motor axle

## Software Architecture (Modular)

| Module | File | Purpose |
|--------|------|---------|
| Camera | `camera.py` | Webcam init, frame capture, resolution config |
| Detector | `detector.py` | HSV-based orange tape detection, centroid + angle computation, overlay drawing |
| Angle Tracker | `angle_tracker.py` | Converts raw ±180° angles to cumulative (unwrapped) angle across multiple revolutions |
| Motor Controller | `motor_controller.py` | Serial communication, speed setting, safety neutral on disconnect |
| Live Plotter | `live_plotter.py` | Real-time scrolling graph of cumulative angle + motor speed (OpenCV-based) |
| Workout Controller | `workout_controller.py` | Calibration, workout modes, closed-loop control, safety limits |
| Logger | `logger.py` | Timestamped text log files in `logs/` directory |
| Workout GUI | `workout_gui.py` | Tkinter GUI: camera feed, graph, mode buttons, calibration, STOP button |
| Spark CAN Utility | `sparkmax_can_gui.py` | OpenCV GUI for SPARK MAX over Arduino CAN bridge (set voltage, view status/current) |
| Main Script | `main.py` | Simpler integration (keyboard only, no workout modes) |
| Detection Test | `test_detection.py` | HSV tuning tool with trackbars, used during initial setup |\n| Settings | `settings.py` | JSON-based settings persistence, auto-save/load |

## SPARK MAX CAN Bridge Utility
- **Arduino sketch**: `Arduino/CAN_SparkMAX_Bridge/CAN_SparkMAX_Bridge.ino`
- **Host GUI**: `sparkmax_can_gui.py`
- **Bridge serial commands**:
	- `VOLTS <device_id> <voltage>`
	- `STOP <device_id>`
	- `HB <device_id> <0|1>`
	- `RATE <device_id> <status_idx> <period_ms>`
	- `TX <id_hex> <dlc> <data_hex>`
- **Bridge serial RX format**: `RX 0x<id_hex> <dlc> <data_hex>`

## Tuned Parameters
- **HSV thresholds**: H [84–129], S [138–255], V [64–213]
- **Rotation center**: (160, 111) in the 320×240 frame
- **Min contour area**: 30 px
- **Circular ROI mask**: 100 px radius centered on rotation center

## Safety Limits
- Motor speed clamped to **neutral ± 20 logical units** (scaled by UNIT≈257 for 16-bit protocol) until user says otherwise
- Motor auto-stops at min/max calibrated angle boundaries (5° margin)
- Near-boundary safety: blocks motor from driving further toward a limit when within 5% of range
- Emergency stop on SPACE, ESC, or STOP button
- Motor stops on quit, Ctrl+C, close, and disconnect

## Calibration Flow
1. Launch `workout_gui.py`
2. Cable at rest → press **Set Min** (or 'm') — resets angle to 0°
3. Pull cable out to full extension → press **Set Max** (or 't')
4. System auto-pulses motor briefly to detect which direction retracts the cable
5. System retracts cable back to start
6. Select a workout mode

## Workout Modes
| Mode | Behavior |
|------|----------|
| **Idle** | Motor off, no control |
| **Constant Speed** | PI control on angular velocity — resists user to enforce a target pull speed |
| **Rubber Band** | Resistance increases linearly with extension (like a rubber band) |
| **Constant Weight** | Constant motor force regardless of position (like a weight stack) |

## GUI Keyboard Shortcuts
| Key | Action |
|-----|--------|
| `m` | Set min position |
| `t` | Set max position |
| `r` | Reset angle to zero |
| `1-4` | Select workout mode (1=Idle, 2=Constant Speed, 3=Rubber Band, 4=Constant Weight) |
| `UP/DOWN` | Increase/decrease power level (0.5 steps) |
| `LEFT/RIGHT` | Decrease/increase target speed (5°/s steps) |
| `SPACE/ESC` | Emergency stop |
| `q` | Quit |

## Controls (main.py — simpler version)
- **W / UP arrow**: Increase motor speed
- **S / DOWN arrow**: Decrease motor speed
- **SPACE**: Stop motor (neutral)
- **R**: Reset cumulative angle to zero
- **Q**: Quit (auto-stops motor)

## Logging
- Session logs written to `logs/session_YYYYMMDD_HHMMSS.log`
- Categories: CALIBRATE, MOTOR, WORKOUT, ANGLE, SAFETY, EVENT, CONTROL
- Angle logged every ~30 frames to avoid file bloat

## Session Log
- **2026-02-28**: Initial setup. Enumerated cameras (index 0 = external webcam, index 1 = MacBook). Built modular detection pipeline. Tuned HSV thresholds interactively. Confirmed detection working with correct center point (160, 111). Built full integrated system with live angle graph and motor control. Confirmed main.py working end-to-end with motor. Built workout GUI with tkinter, workout controller with 3 modes, calibration flow, motor direction auto-detection, safety limits, and text logging. Upgraded serial protocol from 1-byte (0–255) to 2-byte big-endian uint16 (0–65535) for finer motor resolution (~257x more steps). Added settings persistence (JSON auto-save/load), finer power resolution (0.5 steps, float, 0–50 range), user-settable target speed for constant speed mode, improved PI controller with EMA velocity smoothing and idle hold behavior.

## Next Steps
- Test workout_gui.py end-to-end with motor
- Tune PI gains for constant speed mode under real load
- Add rep counting
- Add workout session summary/stats
