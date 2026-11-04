# WP-CB01 dual-control-box ESPHome bridge

Based on Samuel Sieb's [original `jsdrive` ESPHome component](https://github.com/ssieb/esphome_components/tree/jsdrive/components/jsdrive).
See the [repository attribution](../../README.md) and [license](../../LICENSE).

This external component makes an ESP32-S3 an active bridge between one
WP-CB01 handset and two control boxes. Each UART is an independent protocol
session. The bridge sends each peer synthesized state and never forwards one
box's display frames blindly to the handset.

The complete, tested-shape configuration is in
[`wp-cb01-dual.yaml`](../../wp-cb01-dual.yaml). It uses three 9600 8N1 UARTs,
two box action outputs, two box awake inputs, the handset action input, and the
handset awake output.
The example uses this installed board's fixed hostname,
`wp-cb01-dual-2ecf50`, so ESPHome Device Builder can track it. Replace the
literal name if you build another bridge.

## Home Assistant interface

- a cover with position and stop control;
- an absolute target-height number (25.9–51.5, 0.1 steps);
- stop and height-discovery buttons;
- recall and save buttons for presets 1–4;
- confirmed, per-box, and difference height sensors;
- connection, awake, synchronization, live-height, armed, movement, and
  overall-health binary sensors;
- operation, last-fault, per-box display, and decoded-error text sensors;
- frame-age, invalid-frame, 10 ms transmitter-gap, and alignment diagnostics;
- handset Up, Down, and preset-button binary sensors.

The generated YAML actions are `jsdrive2.set_height`, `jsdrive2.stop`,
`jsdrive2.recall_preset`, `jsdrive2.save_preset`, `jsdrive2.discover_height`,
`jsdrive2.arm`, and `jsdrive2.disarm`.

## Movement rules

Movement starts only when both box sessions are awake, responding, and have a
trusted matching position. A missing live height starts the captured wake
handshake; it never uses M as a height query. Absolute moves hold the direction
command, release 0.3 from the target, then use 190 ms fine taps. Presets are
monitored using matched height reports from both boxes, including the matched
destination announcement before physical travel. A physical handset command
has priority over an API move. Handset UP and DOWN can continue across a one
step reporting lag while both boxes remain below the hard skew limit. A held
UP+DOWN chord can recover a disarmed bridge: it holds DOWN continuously on the
higher box until its reported height reaches the lower box, then verifies the
settled heights before rearming. The lower box receives only idle frames.

Each box has its own bounded wake attempt. The bridge keeps an independent
10 ms idle stream to each box and a synthesized status stream to the handset.
The handset display blanks when both box displays time out. Active pin 5 outputs
have a 30 ms command watchdog (60 ms during the continuous recovery), and a
lost UART link or awake signal lowers
the outputs. A normal first-button wake does not show SOS while fresh heights
are pending. Simultaneous button masks are never sent to either box.

A 0.5 height split, link loss, transaction timeout, or invalid travel state
lowers both action outputs, reports the reason, displays SOS on the handset,
and disarms the bridge where continued movement would be unsafe. Small stopped
offsets are corrected by bounded one-sided DOWN taps to the higher box only
after both boxes supply fresh heights. Re-arming requires both box links to be
healthy and settled, and resets the correction attempt counter.

GPIO48 is a 3.3 V logic output into the SN74AHCT125N. The example wiring uses
pin 1 (`1OE`) to ground, GPIO48 to pin 2 (`1A`), and pin 3 (`1Y`) to handset
pin 8. It is HIGH from boot so the handset session can initialize independently
of both physical box sessions.

The component compiles with ESPHome 2026.10.2 and its host protocol tests run
with `bash tests/host/run.sh` from the repository root. The host tests cover
quick repeated presses, watchdog release, the hard skew stop, preset height
pairing, handset display timeout, and independent wake. The ESPHome firmware
has not yet been flashed to the installed desk, so those behaviors still need
hardware validation before replacing the debug firmware.
