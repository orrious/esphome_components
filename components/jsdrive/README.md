# WP-CB01 dual-control-box ESPHome bridge

This external component makes an ESP32-S3 an active bridge between one
WP-CB01 handset and two control boxes. Each UART is an independent protocol
session. The bridge sends each peer synthesized state and never forwards one
box's display frames blindly to the handset.

The complete, tested-shape configuration is in
[`wp-cb01-dual.yaml`](../../wp-cb01-dual.yaml). It uses three 9600 8N1 UARTs,
two box action outputs, two box awake inputs, the handset action input, and the
handset awake output.

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

The generated YAML actions are `jsdrive.set_height`, `jsdrive.stop`,
`jsdrive.recall_preset`, `jsdrive.save_preset`, `jsdrive.discover_height`,
`jsdrive.arm`, and `jsdrive.disarm`.

## Movement rules

Movement starts only when both box sessions are awake, responding, and have a
trusted matching position. A missing live height starts the captured wake
handshake; it never uses M as a height query. Absolute moves hold the direction
command, release 0.3 from the target, then use 190 ms fine taps. Presets are
monitored using matched height reports from both boxes. A physical handset
command has priority over an API move.

A 0.5 height split, link loss, transaction timeout, or invalid travel state
lowers both action outputs, reports the reason, displays SOS on the handset,
and disarms the bridge where continued movement would be unsafe. Small stopped
offsets are corrected by bounded one-sided DOWN pulses to the higher box.

GPIO48 is a 3.3 V logic output into the SN74AHCT125N. The example wiring uses
pin 1 (`1OE`) to ground, GPIO48 to pin 2 (`1A`), and pin 3 (`1Y`) to handset
pin 8. It is HIGH only when both independent box pin-8 inputs are HIGH.
