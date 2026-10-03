import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import binary_sensor, button, number, sensor, uart
from esphome.const import CONF_ID, CONF_HEIGHT

DEPENDENCIES = ['uart']

AUTO_LOAD = ['sensor', 'binary_sensor', 'button', 'number']

jsdrive_ns = cg.esphome_ns.namespace('jsdrive')

JSDrive = jsdrive_ns.class_('JSDrive', cg.Component)
JSDriveTargetNumber = jsdrive_ns.class_('JSDriveTargetNumber', number.Number)
JSDriveStopButton = jsdrive_ns.class_('JSDriveStopButton', button.Button)

CONF_REMOTE_UART = "remote_uart"
CONF_DESK_UART = "desk_uart"
CONF_DESK_UART_A = "desk_uart_a"
CONF_DESK_UART_B = "desk_uart_b"
CONF_MESSAGE_LENGTH = "message_length"
CONF_REMOTE_ACTION_PIN = "remote_action_pin"
CONF_BOX_A_ACTION_PIN = "box_a_action_pin"
CONF_BOX_B_ACTION_PIN = "box_b_action_pin"
CONF_BOX_A_AWAKE_PIN = "box_a_awake_pin"
CONF_BOX_B_AWAKE_PIN = "box_b_awake_pin"
CONF_REMOTE_AWAKE_PIN = "remote_awake_pin"
CONF_UP = "up"
CONF_DOWN = "down"
CONF_MEMORY1 = "memory1"
CONF_MEMORY2 = "memory2"
CONF_MEMORY3 = "memory3"
CONF_HEIGHT_A = "height_a"
CONF_HEIGHT_B = "height_b"
CONF_IN_SYNC = "in_sync"
CONF_TARGET_HEIGHT = "target_height"
CONF_STOP = "stop"

CONFIG_SCHEMA = cv.COMPONENT_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(JSDrive),
    cv.Optional(CONF_REMOTE_UART): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_DESK_UART): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_DESK_UART_A): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_DESK_UART_B): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_MESSAGE_LENGTH, default=5): cv.int_range(min=5, max=6),
    cv.Optional(CONF_REMOTE_ACTION_PIN): pins.gpio_input_pin_schema,
    cv.Optional(CONF_BOX_A_ACTION_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_BOX_B_ACTION_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_BOX_A_AWAKE_PIN): pins.gpio_input_pin_schema,
    cv.Optional(CONF_BOX_B_AWAKE_PIN): pins.gpio_input_pin_schema,
    cv.Optional(CONF_REMOTE_AWAKE_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_HEIGHT): sensor.sensor_schema(
        accuracy_decimals = 1
    ),
    cv.Optional(CONF_HEIGHT_A): sensor.sensor_schema(accuracy_decimals=1),
    cv.Optional(CONF_HEIGHT_B): sensor.sensor_schema(accuracy_decimals=1),
    cv.Optional(CONF_IN_SYNC): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_TARGET_HEIGHT): number.number_schema(JSDriveTargetNumber),
    cv.Optional(CONF_STOP): button.button_schema(JSDriveStopButton),
    cv.Optional(CONF_UP): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_DOWN): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_MEMORY1): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_MEMORY2): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_MEMORY3): binary_sensor.binary_sensor_schema(),
})


async def to_code(config):
    if CONF_DESK_UART in config and (CONF_DESK_UART_A in config or CONF_DESK_UART_B in config):
        raise cv.Invalid("Use desk_uart or desk_uart_a/desk_uart_b, not both")
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_REMOTE_UART in config:
        remote_uart = await cg.get_variable(config[CONF_REMOTE_UART])
        cg.add(var.set_remote_uart(remote_uart))
    if CONF_DESK_UART in config:
        desk_uart = await cg.get_variable(config[CONF_DESK_UART])
        cg.add(var.set_desk_uart_a(desk_uart))
    if CONF_DESK_UART_A in config:
        desk_uart = await cg.get_variable(config[CONF_DESK_UART_A])
        cg.add(var.set_desk_uart_a(desk_uart))
    if CONF_DESK_UART_B in config:
        desk_uart = await cg.get_variable(config[CONF_DESK_UART_B])
        cg.add(var.set_desk_uart_b(desk_uart))
    cg.add(var.set_message_length(config[CONF_MESSAGE_LENGTH]))
    if CONF_HEIGHT in config:
        sens = await sensor.new_sensor(config[CONF_HEIGHT])
        cg.add(var.set_height_sensor(sens))
    if CONF_UP in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_UP])
        cg.add(var.set_up_bsensor(sens))
    if CONF_DOWN in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_DOWN])
        cg.add(var.set_down_bsensor(sens))
    if CONF_MEMORY1 in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_MEMORY1])
        cg.add(var.set_memory1_bsensor(sens))
    if CONF_MEMORY2 in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_MEMORY2])
        cg.add(var.set_memory2_bsensor(sens))
    if CONF_MEMORY3 in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_MEMORY3])
        cg.add(var.set_memory3_bsensor(sens))
    if CONF_HEIGHT_A in config:
        sens = await sensor.new_sensor(config[CONF_HEIGHT_A])
        cg.add(var.set_height_a_sensor(sens))
    if CONF_HEIGHT_B in config:
        sens = await sensor.new_sensor(config[CONF_HEIGHT_B])
        cg.add(var.set_height_b_sensor(sens))
    if CONF_IN_SYNC in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_IN_SYNC])
        cg.add(var.set_in_sync_sensor(sens))
    if CONF_TARGET_HEIGHT in config:
        target = await number.new_number(config[CONF_TARGET_HEIGHT], min_value=20.0, max_value=60.0, step=0.1)
        cg.add(target.set_parent(var))
    if CONF_STOP in config:
        stop = await button.new_button(config[CONF_STOP])
        cg.add(stop.set_parent(var))
    for key, setter, schema in (
        (CONF_REMOTE_ACTION_PIN, "set_remote_action_pin", pins.gpio_input_pin_schema),
        (CONF_BOX_A_ACTION_PIN, "set_box_a_action_pin", pins.gpio_output_pin_schema),
        (CONF_BOX_B_ACTION_PIN, "set_box_b_action_pin", pins.gpio_output_pin_schema),
        (CONF_BOX_A_AWAKE_PIN, "set_box_a_awake_pin", pins.gpio_input_pin_schema),
        (CONF_BOX_B_AWAKE_PIN, "set_box_b_awake_pin", pins.gpio_input_pin_schema),
        (CONF_REMOTE_AWAKE_PIN, "set_remote_awake_pin", pins.gpio_output_pin_schema),
    ):
        if key in config:
            pin = await cg.gpio_pin_expression(config[key])
            cg.add(getattr(var, setter)(pin))
