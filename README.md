# ublox-watchdog

A pure C (C11), zero-dependency library and daemon that connects to a
u-blox **ZED-F9K** GNSS/INS receiver (on the **C100-F9K** evaluation
board) over a USB serial port, decodes the **UBX** binary protocol, and
adds a layer of **GNSS/INS fusion supervision** that existing tools
(u-center, the official ubxlib, Arduino libraries) don't provide.

## Why this project exists

Existing tools already know how to parse and **display** UBX-NAV-PVT,
UBX-ESF-STATUS and UBX-ESF-MEAS messages. That's not the point of this
project.

The value added here is a **programmatic decision layer** on top of raw
parsing:

1. A state machine that tracks GNSS/INS fusion transitions
   (calibration, fusion, degradation, loss) with proper time-based logic
   (configurable timeouts).
2. A drift estimator that integrates speed/IMU data during GNSS
   degradation phases, to quantify "how many meters have we been
   navigating on pure dead reckoning" (not yet implemented — see
   [Status](#status) below).
3. A structured output (JSON over stdout or a Unix socket) meant to be
   consumed by another process (e.g. a ROS2 node), not read by a human
   in a GUI (not yet implemented).

If a feature request boils down to "parse a UBX message and print it",
it's out of scope — that already exists elsewhere.

## Environment

- **Language**: C11 (`-Wall -Wextra -Wpedantic -std=c11`), zero external
  dependencies — no libubx, no libserialport, no cJSON.
- **POSIX only**: `termios.h` for the serial port, `clock_gettime`
  (`CLOCK_MONOTONIC`) for timing, `sys/socket.h` planned for the Unix
  socket output. No Jetson-specific libraries.
- **Target**: embedded Linux (developed and tested on an NVIDIA
  Jetson), but should run on any Linux box with a free USB port.
- **Hardware**: u-blox ZED-F9K on a C100-F9K evaluation board, USB
  cable to the host. Default boot baud rate is 9600.
- **No malloc/free in the hot path**: fixed-size buffers allocated once
  at startup.
- **Build system**: a plain Makefile, no CMake.

## Hardware notes (lessons learned the hard way)

- **The C100-F9K board has two connectors both silkscreened "RF IN"**.
  Only one of them is the primary GNSS antenna input. Plugging the
  antenna into the wrong one looks exactly like a dead antenna: the
  port opens fine, NMEA/UBX traffic flows, but `numSV` stays at 0
  forever. If you get zero satellites after several minutes outdoors
  with a known-good active antenna (e.g. the ANN-MB-00-00), try the
  other RF IN port before suspecting anything else.
- **Periodic UBX push (`CFG-MSGOUT-*`) didn't trigger spontaneous
  output on our unit**, even though the `UBX-CFG-VALSET` command was
  correctly ACKed. Polling (`UBX-NAV-PVT`/`UBX-ESF-STATUS` with an empty
  payload) always works and returns an immediate response. `main.c`
  therefore actively polls both messages at 1 Hz instead of relying on
  the receiver to push them — see `src/ubx_poll.c`. The CFG-VALSET
  machinery (`src/ubx_cfg.c`, `src/setup_messages.c`) is still in the
  tree, tested, and harmless to call, in case this turns out to be
  fix-dependent and starts working once a fix is acquired for longer.
- **`UBX-ESF-STATUS.fusionMode` only leaves `INIT` after a real
  GNSS-correlated motion**, not from shaking the board in place. The
  ZED-F9K's automatic IMU-mount alignment needs translational movement
  with varying speed and a couple of turns (think: walking with the rig
  for a minute, or driving) so it can correlate GNSS-derived velocity
  with the IMU. Watch the per-sensor `calibStatus` field in
  `UBX-ESF-STATUS` for calibration progress before `fusionMode` itself
  flips to `FUSION`.
- The Jetson user generally isn't in the `dialout` group by default; if
  `/dev/ttyACM0` open fails with `Permission denied`, either
  `sudo usermod -aG dialout <user>` and re-login, or run commands via
  `sg dialout -c '...'` in the meantime.

## Building

```sh
make            # builds build/ublox-watchdog
make test       # builds and runs both unit test binaries
make clean
```

## Running

```sh
./build/ublox-watchdog [/dev/ttyACM0] [--degraded-timeout=<ms>]
```

Defaults to `/dev/ttyACM0` if no device path is given, and to a 30000ms
DEGRADED timeout if `--degraded-timeout` isn't passed (order of the two
arguments doesn't matter). The program opens the port at 9600 baud,
then polls `UBX-NAV-PVT` and `UBX-ESF-STATUS` once per second, printing
each received frame in hex, a decoded human-readable summary for those
two message types, and the current fusion state on every cycle. Runs
until interrupted with `Ctrl+C` (SIGINT), which closes the serial port
cleanly before exiting.

## Project layout

```
ublox-watchdog/
├── src/
│   ├── serial_port.c/h      # termios setup for the USB serial port (raw mode, baud rate)
│   ├── ubx_checksum.c/h     # shared UBX Fletcher-8 checksum (used by parser and builders)
│   ├── ubx_parser.c/h       # generic UBX frame state machine (sync bytes, class/ID/length/checksum)
│   ├── ubx_protocol.h/.c    # UBX-NAV-PVT, UBX-ESF-STATUS, UBX-ACK-ACK/NAK structures & decoding
│   ├── ubx_poll.c/h         # builds empty-payload "poll request" frames
│   ├── ubx_cfg.c/h          # builds UBX-CFG-VALSET frames (config-by-key)
│   ├── setup_messages.c/h   # sends a CFG-VALSET and waits for ACK/NAK with a timeout
│   ├── ubx_fusion_tracker.c/h # GNSS/INS fusion state machine (see below)
│   └── main.c                # serial port -> poll loop -> parse -> decode -> print
├── tests/
│   ├── test_ubx_parser.c     # generic framing tests + real captured NAV-PVT/ESF-STATUS fixtures
│   ├── test_fusion_tracker.c # all 5 fusion state machine scenarios, no hardware needed
│   └── fixtures/             # raw UBX frames captured on real hardware
├── examples/
│   └── ros2_bridge_demo/     # planned: minimal Python demo reading the Unix socket from ROS2
├── docs/
│   └── test_protocol.md      # planned: field test protocol (deliberate GNSS cutoff, real drift measurement)
├── Makefile
├── README.md
└── CLAUDE.md                 # full project context/spec for AI-assisted development
```

## The fusion state machine (`ubx_fusion_tracker.c/h`)

States: `UNKNOWN`, `NOT_CALIBRATED`, `CALIBRATING`, `FUSION`, `DEGRADED`, `LOST`.

```
UNKNOWN         --[first NAV-PVT, no fix]-->        NOT_CALIBRATED
UNKNOWN         --[first NAV-PVT, fix valid]-->      CALIBRATING
NOT_CALIBRATED  --[fix becomes valid]-->              CALIBRATING
CALIBRATING     --[fusionMode == FUSION, fix valid]--> FUSION
FUSION          --[fixType < 3 or gnssFixOK == 0]-->  DEGRADED
DEGRADED        --[fix valid again]-->                FUSION
DEGRADED        --[longer than degraded_timeout_ms]--> LOST
LOST            --[fix valid again]-->                CALIBRATING   (re-calibration required)
```

`fusion_tracker_update()` accepts `NULL` for either the NAV-PVT or the
ESF-STATUS pointer (they arrive as separate UBX frames) and never
crashes in that case; it caches the last known fix status internally so
partial updates still work. Each transition is logged to stderr with a
timestamp and a human-readable reason, and is also surfaced in
`main.c`'s own output as a `>>> TRANSITION: ... -> ... (raison: ...)`
line, alongside a per-cycle `[fusion_state: ... | depuis Ns | fix=... numSV=...]`
status line. The degraded timeout is configurable from the command
line via `--degraded-timeout=<ms>` (defaults to 30000).

The drift estimator must never present `accumulated_drift_m` as a
measured position error — it's a distance traveled under pure dead
reckoning, not a verified error. (Documented here ahead of time since
it's a recurring point of confusion; enforced once `drift_estimator.c`
exists.)

## Status

| Component | Status |
|---|---|
| `serial_port.c/h` | Done, tested on real hardware |
| `ubx_parser.c/h` (generic framing) | Done, unit tested |
| `ubx_protocol.c/h` (NAV-PVT, ESF-STATUS, ACK) | Done, unit tested against real captured frames |
| `ubx_cfg.c/h`, `ubx_poll.c/h`, `setup_messages.c/h` | Done, tested on real hardware |
| `ubx_fusion_tracker.c/h` | Done, unit tested (5 scenarios), wired into `main.c`, observable in real-time output |
| `drift_estimator.c/h` | Not started |
| `output_stream.c/h` (JSON / Unix socket) | Not started |
| `examples/ros2_bridge_demo/` | Not started |
| `docs/test_protocol.md` | Not started |

See `CLAUDE.md` for the full project specification and conventions.
