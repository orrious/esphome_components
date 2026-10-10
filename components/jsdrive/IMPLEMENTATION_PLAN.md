# WP-CB01 dual-box ESPHome implementation plan

## Goal

Move the validated three-session bridge from `debug-firmware` into the
`jsdrive` ESPHome external component without weakening its synchronization and
fail-safe behavior. The handset, Box A, and Box B remain independent protocol
peers. The component synthesizes the state each peer needs; it never blindly
relays one box's frames to another peer.

## 1. Port the validated protocol engine

Replace the current prototype loop with the state machines already exercised
in `debug-firmware`:

- independent five-byte UART parsers and link timers for the handset and both
  boxes;
- independent box awake, display, current-height, last-good-height, error, and
  connection state;
- one 10 ms idle transmitter per box and one 10 ms synthesized handset-status
  transmitter;
- handset command ownership with immediate human priority over an HA command;
- 0.5-unit hard skew stop and SOS display;
- post-stop one-sided DOWN alignment of the higher box;
- preset announcement, travel, ordered height pairing, cancellation, and
  retarget behavior;
- target-height coarse hold, release 0.3 from the target, 190 ms fine taps,
  settling checks, and a 45-second timeout;
- independent startup/wake discovery using the captured wake handshake, never
  M as a height query;
- remote pin 8 HIGH only while both box pin-8 inputs are HIGH.

All movement methods return a result/reason and converge through one command
arbiter. No entity implementation writes UARTs or action GPIOs directly.

## 2. Define the HA-facing state model

### Primary controls

- `cover`: normalized desk position with open/close/stop and position control.
- `number`: absolute target height, 25.9–51.5 in 0.1 steps.
- `button`: Stop.
- `button`: Recall Preset 1, 2, 3, and 4.
- `button`: Save Preset 1, 2, 3, and 4. Saving is an M transaction: send M to
  both boxes, require matching memory prompts, send the selected preset to both,
  and fail cleanly if either side disagrees or times out.

### Position and synchronization

- confirmed desk height;
- Box A and Box B last-good heights;
- live height difference;
- target height;
- moving direction/operation;
- synchronized binary sensor;
- height-known and height-live binary sensors. Display-off keeps the confirmed
  cached height; reboot begins unknown until discovery succeeds.

### Link and health diagnostics

- overall healthy/ready binary sensor;
- handset connected;
- Box A connected and Box B connected;
- Box A awake and Box B awake;
- armed and pass-through state;
- per-UART age since last valid frame;
- invalid-frame/checksum counters;
- maximum transmitter gap for each 10 ms stream;
- alignment attempt count and active transaction phase;
- last stop/fault reason as a text sensor.

### Display and errors

- Box A and Box B display-state text sensors: height, blank, memory prompt,
  startup test, known error, or unknown raw glyphs;
- decoded error text sensor, including observed E04 and bridge-generated SOS;
- raw display bytes retained in diagnostic logs for unknown codes;
- handset receives a synthesized agreed height, memory prompt, error, or SOS.

## 3. ESPHome component API

Add generated ESPHome actions so YAML automations and the native API can call:

- `jsdrive.set_height` with a templatable height;
- `jsdrive.stop`;
- `jsdrive.recall_preset` with preset 1–4;
- `jsdrive.save_preset` with preset 1–4;
- `jsdrive.arm` and `jsdrive.disarm` for diagnostics;
- `jsdrive.discover_height` using the wake handshake.

Buttons and the target-height number call these same actions. The component
publishes failures through state and logs rather than silently ignoring them.

## 4. YAML and hardware configuration

Use three ESP32-S3 hardware UARTs at 9600 8N1. Logging remains on USB
Serial/JTAG so it consumes no desk UART. The checked-in full YAML uses the
validated wiring:

| Peer | ESP RX | ESP TX | Action | Awake |
| --- | ---: | ---: | ---: | ---: |
| Box A | GPIO4 | GPIO5 | GPIO9 | GPIO41 |
| Box B | GPIO15 | GPIO16 | GPIO11 | GPIO40 |
| Handset | GPIO2 | GPIO1 | GPIO39 input | GPIO48 output |

GPIO48 drives SN74AHCT125N pin 2 (`1A`), pin 1 (`1OE`) is grounded, and pin 3
(`1Y`) drives handset pin 8.

## 5. Delivery sequence

1. Extract/port frame parsing, display decoding, independent sessions, and
   synthesized handset status. Compile with ESPHome and validate with no motors.
2. Port handset pass-through, wake discovery, link health, and emergency stop.
3. Port presets and save-memory transactions; validate handset and HA invoke the
   same state machines.
4. Port target-height coarse/fine control and alignment policy.
5. Add all sensors, text sensors, buttons, cover, actions, and diagnostic
   counters.
6. Run disconnected protocol tests, then one-box no-motion tests, then two-box
   synchronized movement tests with the 0.5 hard stop enabled throughout.
7. Commit and push the branch used by `external_components`, pin the YAML to a
   tested commit or release tag, and remove the one-day moving branch refresh.

## Acceptance criteria

- Handset and HA controls behave identically at the box-facing sessions.
- Either box can sleep, wake, disappear, or report an error independently.
- No movement begins without both box sessions ready and a trusted synchronized
  position.
- A display-off frame does not erase a confirmed matching position.
- Any 0.5-unit split, stale link, invalid transaction, timeout, or explicit stop
  lowers both action outputs and reports a visible reason.
- Unknown display codes are preserved and exposed rather than discarded.
