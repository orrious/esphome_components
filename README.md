# WP-CB01 ESPHome bridge

This repository contains the [`jsdrive2` ESPHome external component](components/jsdrive2)
for one handset and two WP-CB01 desk control boxes. The boxes and handset have
independent UART sessions; the bridge monitors both box heights and stops motion
when their positions diverge beyond the configured limit.

The [example configuration](wp-cb01-dual.yaml) shows the ESP32-S3 wiring and
Home Assistant entities. It pulls `jsdrive2` from the `wp-cb01-dual-box` branch.

## Attribution

`jsdrive2` began as a modification of Samuel Sieb's
[`jsdrive` ESPHome component](https://github.com/ssieb/esphome_components/tree/jsdrive/components/jsdrive).
The original component supplied five-byte handset-command parsing, desk display
decoding, and the initial ESPHome component structure. The WP-CB01 dual-box
protocol sessions, synchronization, and Home Assistant interface were developed
for this project. The original Git history and the repository [license](LICENSE)
are retained.
