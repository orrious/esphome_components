# ESPHome component for a JS-Drive / WP-CB01 desk bridge

The component can bridge one handset to two synchronized control boxes. It
forwards the validated 5-byte `A5` commands to both boxes, forwards one
validated display response back to the handset, decodes each box's height, and
publishes an in-sync binary sensor. The GPIOs are optional so the UART-only
bridge can be brought up first.

Example:
```yaml
uart:
  - id: remote_bus
    rx_pin: 2
    tx_pin: 1
    baud_rate: 9600
  - id: box_a_bus
    rx_pin: 4
    tx_pin: 5
    baud_rate: 9600
  - id: box_b_bus
    rx_pin: 15
    tx_pin: 16
    baud_rate: 9600

jsdrive:
  id: my_jsdrive
  remote_uart: remote_bus
  desk_uart_a: box_a_bus
  desk_uart_b: box_b_bus
  message_length: 5
  remote_action_pin: 39
  box_a_action_pin: 9
  box_b_action_pin: 11
  box_a_awake_pin: 41
  box_b_awake_pin: 40
  # GPIO48 -> SN74AHCT125N 1A; 1OE is grounded; 1Y -> handset pin 8.
  remote_awake_pin: 48
  height:
    name: Desk Height
  height_a:
    name: Box A Height
  height_b:
    name: Box B Height
  in_sync:
    name: Control Boxes In Sync
  target_height:
    name: Target Desk Height
  stop:
    name: Stop Desk
  up:
    name: Up Button
  down: 
    name: Down Button
  memory1:
    name: Memory1 Button
  memory2:
    name: Memory2 Button
  memory3:
    name: Memory3 Button
```

`height`, `height_a`, and `height_b` are sensors. `in_sync` is true only when
both boxes have valid numeric display packets and differ by no more than 0.1.
The button entities are binary sensors indicating handset command states.
`target_height` and `stop` expose Home Assistant control without requiring a
physical handset press. Target movement asserts both box action lines while
periodically sending the corresponding Up or Down command.

There are methods `move_to(height)` and `stop()` that you can use in a lambda.
