# WP-CB01 dual-box diagnostic bridge

This is a breadboard-first firmware for validating the ESP32-S3 wiring before
using the ESPHome component. It mirrors validated handset commands to both
control boxes, returns one display response to the handset, mirrors the action
line, monitors sleep state, and exposes a small USB serial test console.

## Build and flash

Install PlatformIO, connect the native USB port configured for USB CDC, then:

```text
pio run -t upload
pio device monitor
```

The second USB connection can remain attached for power or the other serial
interface. The diagnostic console is 115200 baud.

## Wiring used by this project

| Signal | ESP32 GPIO |
|---|---:|
| Remote UART RX/TX | 12 / 13 |
| Control box A RX/TX | 4 / 5 |
| Control box B RX/TX | 8 / 9 |
| Remote action input | 14 |
| Box A action output | 6 |
| Box B action output | 10 |
| Box A awake input | 7 |
| Box B awake input | 11 |
| Remote awake open-drain output | 15 |

All desk signals must remain behind the 5 V level shifters. Pin 10 of the
WP-CB01 connector is the approximately 35 V supply and stays isolated.

## Console commands

```text
help
status
up                 # one short action pulse
down
stop
m
preset 1           # preset 1 through 4 are supported
pulse 500          # change the default action pulse duration
pass on|off        # enable or disable handset mirroring
raw on|off         # enable or disable frame logging
raw a5 00 20 df ff # send a validated command to both boxes
```

The firmware starts disarmed, with action outputs low and pass-through
disabled. It logs every received byte as `BYTE REMOTE_RX`, `BYTE BOX_A_RX`, or
`BYTE BOX_B_RX`, which lets us see startup traffic without forwarding it.
Begin testing with the remote disconnected and one control box connected. Use
`status` and inspect the byte log before typing `arm` or enabling pass-through.
Add the second box only after the first box responds correctly.
