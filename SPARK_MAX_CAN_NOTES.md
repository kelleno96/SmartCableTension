# SPARK MAX CAN Protocol Notes

Empirically verified against a SPARK MAX running current firmware (shifted ID scheme).
All CAN frames are 29-bit extended.

---

## CAN ID Structure

```
bits [28:24] = Device Type  (2 = motor controller)
bits [23:16] = Manufacturer (5 = REV Robotics)
bits [15:10] = API Class    (6 bits)
bits  [9:6]  = API Index    (4 bits)
bits  [5:0]  = Device ID    (6 bits, 1–62)
```

`api10 = (apiClass << 4) | apiIndex` — a 10-bit combined API identifier.

### Shifted vs Legacy IDs

Newer SPARK MAX firmware broadcasts status frames using **shifted** API10 values (`api10 + 0x280`).
Always normalize before comparing: `unshifted = api10 >= 0x280 ? api10 - 0x280 : api10`.

Control commands (sent TO the controller) also have a shifted variant.
The Arduino bridge sends both to cover all firmware versions.

---

## Control Commands (Host → SPARK MAX)

All setpoint frames have identical payload structure: 8 bytes.

| Bytes | Content                                                   |
|-------|-----------------------------------------------------------|
| 0–3   | float32 LE setpoint                                       |
| 4–5   | int16 LE arbitrary feedforward (0 = disabled)             |
| 6     | bits [1:0] = PID slot (0–3); bits [7:2] = reserved        |
| 7     | reserved                                                  |

**Each control mode uses its own API ID** (spark-frames-2.1.0 spec confirmed):

| Command | api10  | Setpoint units  | Notes                         |
|---------|--------|-----------------|-------------------------------|
| `VEL`   | `0x000`| RPM             | Closed-loop, needs PID slot 1 |
| `DUTY`  | `0x002`| −1.0 .. 1.0     | Open-loop, no PID needed      |
| `POS`   | `0x004`| rotations       | Closed-loop, needs PID slot 0 |
| `VOLTS` | `0x005`| volts (±12)     | Closed-loop voltage           |

CAN ID = `makeSparkIdApi10(api10, deviceId)`

> **Previous incorrect assumption**: We sniffed Hardware Client traffic while only testing
> velocity and mistakenly concluded all modes shared api10=`0x000`. This caused position
> commands to be sent as velocity commands (e.g. POS 5.0 → 5 RPM with position PID gains),
> producing oscillation. The spec (spark-frames-2.1.0) confirmed separate IDs per mode.

> **PID slots (this project)**: slot 0 = position gains, slot 1 = velocity gains.

### Heartbeat

Must be sent every ~20 ms or the SPARK MAX disables.

**Non-RIO heartbeat** (for standalone / non-FRC use) — **verified from traffic**:
- API10 `0x0B2`, device ID **0** (broadcast, NOT device-specific)
- CAN ID = `makeSparkIdApi10(0x0B2, 0)` = `0x02052C80`
- Payload: 64-bit bitfield — bit N = 1 enables device with CAN ID N
  - For device ID 1: `[0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]`
  - Observed from Hardware Client: exactly this format

> **Old heartbeat (0x058 / 0x2D8, device-specific) does NOT work on firmware 26.x.**
> The motor will remain disabled and ignore all commands if the wrong heartbeat is sent.

---

## Status Frames (SPARK MAX → Host, periodic)

Sent by the SPARK MAX automatically. Set update rate with `RATE <devId> <idx> <ms>`.
Default periods: Status 0 = 10 ms, Status 1 = 20 ms, Status 2 = 20 ms, Status 3+ = disabled (0).

### Status 0 — Applied Output & Faults
`api10 = 0x060` (shifted `0x2E0`)

| Bytes | Type      | Content                           |
|-------|-----------|-----------------------------------|
| 0–1   | int16 LE  | Applied output (÷ 32767 = −1..1)  |
| 2–3   | uint16 LE | Active faults bitmask             |
| 4–5   | uint16 LE | Sticky faults bitmask             |
| 6–7   | —         | Flags (invert, brake, follower)   |

### Status 1 — Faults & Warnings
`api10 = 0x061` (shifted `0x2E1`)

All 8 bytes are fault/warning bitfields (active faults, active warnings, sticky faults,
sticky warnings, follower flag). **Not a telemetry frame.** Velocity/temp/voltage/current
are NOT here — see Status 2 for velocity and Status 0 for applied output.

> We do not currently decode Status 1 in the bridge.

### Status 2 — Encoder Position & Velocity ✓ verified
`api10 = 0x062` (shifted `0x2E2`)

| Bytes | Type       | Content                            |
|-------|------------|------------------------------------|
| 0–3   | float32 LE | **Velocity (RPM)**                 |
| 4–7   | float32 LE | **Position (rotations, relative)** |

> Confirmed empirically: bytes 0–3 track velocity, bytes 4–7 accumulate rotations.

### Status 3 — Analog Sensor ✓ verified
`api10 = 0x063` (shifted `0x2E3`)

| Bytes | Type       | Content                                         |
|-------|------------|-------------------------------------------------|
| 0–1   | 2q8 fixed  | ADC voltage (raw)                               |
| 2–4   | 15q7 fixed | Analog velocity                                 |
| 4–7   | float32 LE | **Analog position (0.0–1.0 per revolution)**    |

> Our absolute encoder is wired to the SPARK MAX analog input.
> Observed ~0.299 at rest — stable, low-noise reading.
> This is NOT Status 5 (duty-cycle). Encoder is read as analog, not PWM.

### Status 4 — Alternate Encoder
`api10 = 0x064` (shifted `0x2E4`)

| Bytes | Type       | Content                        |
|-------|------------|--------------------------------|
| 0–3   | float32 LE | Alt encoder velocity (RPM)     |
| 4–7   | float32 LE | Alt encoder position (rotations) |

### Status 5 — Duty Cycle Absolute Encoder
`api10 = 0x065` (shifted `0x2E5`)

| Bytes | Type       | Content                              |
|-------|------------|--------------------------------------|
| 0–3   | float32 LE | Absolute position (0.0–1.0 per rev)  |
| 4–7   | float32 LE | Absolute encoder velocity            |

> Not observed in our traffic — encoder is on analog input (Status 3), not data port PWM.

---

## Bridge Serial Protocol

Implemented in `Arduino/CAN_SparkMAX_Bridge/CAN_SparkMAX_Bridge.ino`.

### Commands (PC → Arduino)

```
VEL   <devId> <rpm> [slot]       Velocity closed-loop, api10=0x000, default slot=1
POS   <devId> <rotations> [slot] Position closed-loop, api10=0x004, default slot=0
DUTY  <devId> <-1..1>            Duty cycle open-loop, api10=0x002
VOLTS <devId> <voltage>          Voltage control ±12 V, api10=0x005
STOP  <devId>                    Zero all control modes (0x000/0x002/0x004/0x005), all slots
HB    <devId> <0|1>              Enable/disable heartbeat
RATE  <devId> <idx> <ms>         Set status frame period (idx 0–6)
TX    <id_hex> <dlc> <data_hex>  Raw CAN frame
PING                             Responds PONG (tests serial link)
```

### Output (Arduino → PC)

```
RX <id_hex> <dlc> <data_hex>  Raw received frame (always emitted)
POS    <devId> <rotations>    Decoded from Status 2 bytes 4–7
VEL    <devId> <rpm>          Decoded from Status 2 bytes 0–3
ABSPOS <devId> <0..1>         Decoded from Status 3 bytes 4–7 (analog encoder)
                              or Status 5 bytes 0–3 (duty-cycle encoder)
OUTPUT <devId> <-1..1>        Decoded from Status 0 bytes 0–1
```

---

## Observed CAN IDs (Device ID 1)

| CAN ID       | api10  | Frame    | Notes                          |
|--------------|--------|----------|--------------------------------|
| `0x0205B801` | `0x2E0` | Status 0 | Applied output + faults        |
| `0x0205B841` | `0x2E1` | Status 1 | Velocity/temp (all zeros observed) |
| `0x0205B881` | `0x2E2` | Status 2 | Velocity + position ✓          |
| `0x0205B8C1` | `0x2E3` | Status 3 | Analog encoder position ✓      |
| `0x0205B901` | `0x2E4` | Status 4 | Alt encoder (zeros, unused)    |
| `0x0205BA41` | `0x2E9` | Status 9 | Unknown, newer firmware only   |
