import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation, pins
from esphome.components import binary_sensor, button, cover, number, sensor, text_sensor, uart
from esphome.const import CONF_HEIGHT, CONF_ID

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor", "button", "cover", "number", "sensor", "text_sensor"]

jsdrive_ns = cg.esphome_ns.namespace("jsdrive")
JSDrive = jsdrive_ns.class_("JSDrive", cg.Component)
JSDriveTargetNumber = jsdrive_ns.class_("JSDriveTargetNumber", number.Number)
JSDriveCover = jsdrive_ns.class_("JSDriveCover", cover.Cover)
JSDriveCommandButton = jsdrive_ns.class_("JSDriveCommandButton", button.Button)
JSDriveSetHeightAction = jsdrive_ns.class_("JSDriveSetHeightAction", automation.Action)
JSDrivePresetAction = jsdrive_ns.class_("JSDrivePresetAction", automation.Action)
JSDriveSimpleAction = jsdrive_ns.class_("JSDriveSimpleAction", automation.Action)

CONF_REMOTE_UART = "remote_uart"
CONF_DESK_UART_A = "desk_uart_a"
CONF_DESK_UART_B = "desk_uart_b"
CONF_REMOTE_ACTION_PIN = "remote_action_pin"
CONF_BOX_A_ACTION_PIN = "box_a_action_pin"
CONF_BOX_B_ACTION_PIN = "box_b_action_pin"
CONF_BOX_A_AWAKE_PIN = "box_a_awake_pin"
CONF_BOX_B_AWAKE_PIN = "box_b_awake_pin"
CONF_REMOTE_AWAKE_PIN = "remote_awake_pin"
CONF_HEIGHT_A = "height_a"
CONF_HEIGHT_B = "height_b"
CONF_HEIGHT_DIFFERENCE = "height_difference"
CONF_REMOTE_FRAME_AGE = "remote_frame_age"
CONF_BOX_A_FRAME_AGE = "box_a_frame_age"
CONF_BOX_B_FRAME_AGE = "box_b_frame_age"
CONF_REMOTE_INVALID_FRAMES = "remote_invalid_frames"
CONF_BOX_A_INVALID_FRAMES = "box_a_invalid_frames"
CONF_BOX_B_INVALID_FRAMES = "box_b_invalid_frames"
CONF_REMOTE_TX_MAX_GAP = "remote_tx_max_gap"
CONF_BOX_A_TX_MAX_GAP = "box_a_tx_max_gap"
CONF_BOX_B_TX_MAX_GAP = "box_b_tx_max_gap"
CONF_ALIGNMENT_ATTEMPTS = "alignment_attempts"
CONF_TARGET_HEIGHT = "target_height"
CONF_COVER = "cover"
CONF_IN_SYNC = "in_sync"
CONF_HEIGHT_KNOWN = "height_known"
CONF_HEIGHT_LIVE = "height_live"
CONF_HEALTHY = "healthy"
CONF_REMOTE_CONNECTED = "remote_connected"
CONF_BOX_A_CONNECTED = "box_a_connected"
CONF_BOX_B_CONNECTED = "box_b_connected"
CONF_BOX_A_AWAKE = "box_a_awake"
CONF_BOX_B_AWAKE = "box_b_awake"
CONF_ARMED = "armed"
CONF_MOVING = "moving"
CONF_OPERATION = "operation"
CONF_LAST_FAULT = "last_fault"
CONF_BOX_A_DISPLAY = "box_a_display"
CONF_BOX_B_DISPLAY = "box_b_display"
CONF_ERROR_CODE = "error_code"
CONF_STOP = "stop"
CONF_DISCOVER_HEIGHT = "discover_height"
CONF_RECALL_PRESET_1 = "recall_preset_1"
CONF_RECALL_PRESET_2 = "recall_preset_2"
CONF_RECALL_PRESET_3 = "recall_preset_3"
CONF_RECALL_PRESET_4 = "recall_preset_4"
CONF_SAVE_PRESET_1 = "save_preset_1"
CONF_SAVE_PRESET_2 = "save_preset_2"
CONF_SAVE_PRESET_3 = "save_preset_3"
CONF_SAVE_PRESET_4 = "save_preset_4"
CONF_UP = "up"
CONF_DOWN = "down"
CONF_MEMORY1 = "memory1"
CONF_MEMORY2 = "memory2"
CONF_MEMORY3 = "memory3"
CONF_MEMORY4 = "memory4"
CONF_MIN_HEIGHT = "min_height"
CONF_MAX_HEIGHT = "max_height"
CONF_HARD_SKEW = "hard_skew"
CONF_PRESET = "preset"

CONFIG_SCHEMA = cv.COMPONENT_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(JSDrive),
    cv.Required(CONF_REMOTE_UART): cv.use_id(uart.UARTComponent),
    cv.Required(CONF_DESK_UART_A): cv.use_id(uart.UARTComponent),
    cv.Required(CONF_DESK_UART_B): cv.use_id(uart.UARTComponent),
    cv.Required(CONF_REMOTE_ACTION_PIN): pins.gpio_input_pin_schema,
    cv.Required(CONF_BOX_A_ACTION_PIN): pins.gpio_output_pin_schema,
    cv.Required(CONF_BOX_B_ACTION_PIN): pins.gpio_output_pin_schema,
    cv.Required(CONF_BOX_A_AWAKE_PIN): pins.gpio_input_pin_schema,
    cv.Required(CONF_BOX_B_AWAKE_PIN): pins.gpio_input_pin_schema,
    cv.Required(CONF_REMOTE_AWAKE_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_MIN_HEIGHT, default=25.9): cv.float_range(min=0),
    cv.Optional(CONF_MAX_HEIGHT, default=51.5): cv.float_range(min=0),
    cv.Optional(CONF_HARD_SKEW, default=0.5): cv.float_range(min=0.1, max=2.0),
    cv.Optional(CONF_HEIGHT): sensor.sensor_schema(accuracy_decimals=1),
    cv.Optional(CONF_HEIGHT_A): sensor.sensor_schema(accuracy_decimals=1),
    cv.Optional(CONF_HEIGHT_B): sensor.sensor_schema(accuracy_decimals=1),
    cv.Optional(CONF_HEIGHT_DIFFERENCE): sensor.sensor_schema(accuracy_decimals=1),
    cv.Optional(CONF_REMOTE_FRAME_AGE): sensor.sensor_schema(unit_of_measurement="ms", accuracy_decimals=0),
    cv.Optional(CONF_BOX_A_FRAME_AGE): sensor.sensor_schema(unit_of_measurement="ms", accuracy_decimals=0),
    cv.Optional(CONF_BOX_B_FRAME_AGE): sensor.sensor_schema(unit_of_measurement="ms", accuracy_decimals=0),
    cv.Optional(CONF_REMOTE_INVALID_FRAMES): sensor.sensor_schema(accuracy_decimals=0),
    cv.Optional(CONF_BOX_A_INVALID_FRAMES): sensor.sensor_schema(accuracy_decimals=0),
    cv.Optional(CONF_BOX_B_INVALID_FRAMES): sensor.sensor_schema(accuracy_decimals=0),
    cv.Optional(CONF_REMOTE_TX_MAX_GAP): sensor.sensor_schema(unit_of_measurement="ms", accuracy_decimals=0),
    cv.Optional(CONF_BOX_A_TX_MAX_GAP): sensor.sensor_schema(unit_of_measurement="ms", accuracy_decimals=0),
    cv.Optional(CONF_BOX_B_TX_MAX_GAP): sensor.sensor_schema(unit_of_measurement="ms", accuracy_decimals=0),
    cv.Optional(CONF_ALIGNMENT_ATTEMPTS): sensor.sensor_schema(accuracy_decimals=0),
    cv.Optional(CONF_TARGET_HEIGHT): number.number_schema(JSDriveTargetNumber),
    cv.Optional(CONF_COVER): cover.cover_schema(JSDriveCover),
    cv.Optional(CONF_IN_SYNC): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_HEIGHT_KNOWN): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_HEIGHT_LIVE): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_HEALTHY): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_REMOTE_CONNECTED): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_BOX_A_CONNECTED): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_BOX_B_CONNECTED): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_BOX_A_AWAKE): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_BOX_B_AWAKE): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_ARMED): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_MOVING): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_OPERATION): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_LAST_FAULT): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_BOX_A_DISPLAY): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_BOX_B_DISPLAY): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_ERROR_CODE): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_UP): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_DOWN): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_MEMORY1): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_MEMORY2): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_MEMORY3): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_MEMORY4): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_STOP): button.button_schema(JSDriveCommandButton),
    cv.Optional(CONF_DISCOVER_HEIGHT): button.button_schema(JSDriveCommandButton),
    cv.Optional(CONF_RECALL_PRESET_1): button.button_schema(JSDriveCommandButton),
    cv.Optional(CONF_RECALL_PRESET_2): button.button_schema(JSDriveCommandButton),
    cv.Optional(CONF_RECALL_PRESET_3): button.button_schema(JSDriveCommandButton),
    cv.Optional(CONF_RECALL_PRESET_4): button.button_schema(JSDriveCommandButton),
    cv.Optional(CONF_SAVE_PRESET_1): button.button_schema(JSDriveCommandButton),
    cv.Optional(CONF_SAVE_PRESET_2): button.button_schema(JSDriveCommandButton),
    cv.Optional(CONF_SAVE_PRESET_3): button.button_schema(JSDriveCommandButton),
    cv.Optional(CONF_SAVE_PRESET_4): button.button_schema(JSDriveCommandButton),
})


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    for key, setter in ((CONF_REMOTE_UART, "set_remote_uart"), (CONF_DESK_UART_A, "set_desk_uart_a"), (CONF_DESK_UART_B, "set_desk_uart_b")):
        cg.add(getattr(var, setter)(await cg.get_variable(config[key])))
    for key, setter in (
        (CONF_REMOTE_ACTION_PIN, "set_remote_action_pin"), (CONF_BOX_A_ACTION_PIN, "set_box_a_action_pin"),
        (CONF_BOX_B_ACTION_PIN, "set_box_b_action_pin"), (CONF_BOX_A_AWAKE_PIN, "set_box_a_awake_pin"),
        (CONF_BOX_B_AWAKE_PIN, "set_box_b_awake_pin"), (CONF_REMOTE_AWAKE_PIN, "set_remote_awake_pin"),
    ):
        cg.add(getattr(var, setter)(await cg.gpio_pin_expression(config[key])))
    cg.add(var.set_limits(config[CONF_MIN_HEIGHT], config[CONF_MAX_HEIGHT]))
    cg.add(var.set_hard_skew(config[CONF_HARD_SKEW]))
    for key, setter in (
        (CONF_HEIGHT, "set_height_sensor"), (CONF_HEIGHT_A, "set_height_a_sensor"),
        (CONF_HEIGHT_B, "set_height_b_sensor"), (CONF_HEIGHT_DIFFERENCE, "set_height_difference_sensor"),
        (CONF_REMOTE_FRAME_AGE, "set_remote_frame_age_sensor"), (CONF_BOX_A_FRAME_AGE, "set_box_a_frame_age_sensor"),
        (CONF_BOX_B_FRAME_AGE, "set_box_b_frame_age_sensor"), (CONF_REMOTE_INVALID_FRAMES, "set_remote_invalid_frames_sensor"),
        (CONF_BOX_A_INVALID_FRAMES, "set_box_a_invalid_frames_sensor"), (CONF_BOX_B_INVALID_FRAMES, "set_box_b_invalid_frames_sensor"),
        (CONF_REMOTE_TX_MAX_GAP, "set_remote_tx_max_gap_sensor"), (CONF_BOX_A_TX_MAX_GAP, "set_box_a_tx_max_gap_sensor"),
        (CONF_BOX_B_TX_MAX_GAP, "set_box_b_tx_max_gap_sensor"), (CONF_ALIGNMENT_ATTEMPTS, "set_alignment_attempts_sensor"),
    ):
        if key in config:
            cg.add(getattr(var, setter)(await sensor.new_sensor(config[key])))
    for key, setter in (
        (CONF_IN_SYNC, "set_in_sync_sensor"), (CONF_HEIGHT_KNOWN, "set_height_known_sensor"),
        (CONF_HEIGHT_LIVE, "set_height_live_sensor"), (CONF_HEALTHY, "set_healthy_sensor"),
        (CONF_REMOTE_CONNECTED, "set_remote_connected_sensor"), (CONF_BOX_A_CONNECTED, "set_box_a_connected_sensor"),
        (CONF_BOX_B_CONNECTED, "set_box_b_connected_sensor"), (CONF_BOX_A_AWAKE, "set_box_a_awake_sensor"),
        (CONF_BOX_B_AWAKE, "set_box_b_awake_sensor"), (CONF_ARMED, "set_armed_sensor"),
        (CONF_MOVING, "set_moving_sensor"), (CONF_UP, "set_up_sensor"), (CONF_DOWN, "set_down_sensor"),
        (CONF_MEMORY1, "set_memory1_sensor"), (CONF_MEMORY2, "set_memory2_sensor"),
        (CONF_MEMORY3, "set_memory3_sensor"), (CONF_MEMORY4, "set_memory4_sensor"),
    ):
        if key in config:
            cg.add(getattr(var, setter)(await binary_sensor.new_binary_sensor(config[key])))
    for key, setter in (
        (CONF_OPERATION, "set_operation_sensor"), (CONF_LAST_FAULT, "set_last_fault_sensor"),
        (CONF_BOX_A_DISPLAY, "set_box_a_display_sensor"), (CONF_BOX_B_DISPLAY, "set_box_b_display_sensor"),
        (CONF_ERROR_CODE, "set_error_code_sensor"),
    ):
        if key in config:
            cg.add(getattr(var, setter)(await text_sensor.new_text_sensor(config[key])))
    if CONF_TARGET_HEIGHT in config:
        target = await number.new_number(config[CONF_TARGET_HEIGHT], min_value=config[CONF_MIN_HEIGHT], max_value=config[CONF_MAX_HEIGHT], step=0.1)
        cg.add(target.set_parent(var))
    if CONF_COVER in config:
        cov = await cover.new_cover(config[CONF_COVER])
        cg.add(cov.set_parent(var))
        cg.add(var.set_cover(cov))
    modes = {CONF_STOP: 0, CONF_DISCOVER_HEIGHT: 1,
             CONF_RECALL_PRESET_1: 11, CONF_RECALL_PRESET_2: 12, CONF_RECALL_PRESET_3: 13, CONF_RECALL_PRESET_4: 14,
             CONF_SAVE_PRESET_1: 21, CONF_SAVE_PRESET_2: 22, CONF_SAVE_PRESET_3: 23, CONF_SAVE_PRESET_4: 24}
    for key, mode in modes.items():
        if key in config:
            btn = await button.new_button(config[key])
            cg.add(btn.set_parent(var))
            cg.add(btn.set_mode(mode))


@automation.register_action("jsdrive.set_height", JSDriveSetHeightAction, cv.Schema({cv.Required(CONF_ID): cv.use_id(JSDrive), cv.Required(CONF_HEIGHT): cv.templatable(cv.float_)}), synchronous=True)
async def set_height_action_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    cg.add(var.set_height(await cg.templatable(config[CONF_HEIGHT], args, cg.float_)))
    return var


PRESET_SCHEMA = cv.Schema({cv.Required(CONF_ID): cv.use_id(JSDrive), cv.Required(CONF_PRESET): cv.templatable(cv.int_range(min=1, max=4))})


def register_preset_action(name, save):
    @automation.register_action(name, JSDrivePresetAction, PRESET_SCHEMA, synchronous=True)
    async def preset_action_to_code(config, action_id, template_arg, args):
        parent = await cg.get_variable(config[CONF_ID])
        var = cg.new_Pvariable(action_id, template_arg, parent)
        cg.add(var.set_preset(await cg.templatable(config[CONF_PRESET], args, cg.uint8)))
        cg.add(var.set_save(save))
        return var


register_preset_action("jsdrive.recall_preset", False)
register_preset_action("jsdrive.save_preset", True)


def register_simple_action(name, mode):
    @automation.register_action(name, JSDriveSimpleAction, cv.Schema({cv.Required(CONF_ID): cv.use_id(JSDrive)}), synchronous=True)
    async def simple_action_to_code(config, action_id, template_arg, args):
        parent = await cg.get_variable(config[CONF_ID])
        var = cg.new_Pvariable(action_id, template_arg, parent)
        cg.add(var.set_mode(mode))
        return var


register_simple_action("jsdrive.stop", 0)
register_simple_action("jsdrive.discover_height", 1)
register_simple_action("jsdrive.arm", 2)
register_simple_action("jsdrive.disarm", 3)
