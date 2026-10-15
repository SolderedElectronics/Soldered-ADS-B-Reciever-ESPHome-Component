import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation, pins
from esphome.components import uart
from esphome.const import CONF_BAUD_RATE, CONF_ID, CONF_RESET_PIN

CODEOWNERS = ["@SolderedElectronics"]
DEPENDENCIES = ["uart"]
MULTI_CONF = True

CONF_ADSB_STATISTICS = "adsb_statistics"
CONF_DECODED_PROTOCOL = "decoded_protocol"
CONF_REPORT_MODE = "report_mode"
CONF_SYSTEM_LOG = "system_log"
CONF_SYSTEM_STATISTICS = "system_statistics"

CONF_ON_ADSB_STATS = "on_adsb_stats"
CONF_ON_ADSB_VEHICLE = "on_adsb_vehicle"
CONF_ON_AIRCRAFT = "on_aircraft"
CONF_ON_LINE = "on_line"
CONF_ON_MAVLINK_FRAME = "on_mavlink_frame"
CONF_ON_SYSTEM_STATS = "on_system_stats"
CONF_ON_UNKNOWN_LINE = "on_unknown_line"

soldered_adsb_receiver_ns = cg.esphome_ns.namespace("soldered_adsb_receiver")
SolderedADSBReceiver = soldered_adsb_receiver_ns.class_(
    "SolderedADSBReceiver", cg.Component, uart.UARTDevice
)

Aircraft = soldered_adsb_receiver_ns.struct("Aircraft")
ADSBStats = soldered_adsb_receiver_ns.struct("ADSBStats")
SystemStats = soldered_adsb_receiver_ns.struct("SystemStats")
ADSBVehicle = soldered_adsb_receiver_ns.struct("ADSBVehicle")
MAVLinkFrame = soldered_adsb_receiver_ns.struct("MAVLinkFrame")

DecodedProtocol = soldered_adsb_receiver_ns.enum("DecodedProtocol")
DECODED_PROTOCOL_OPTIONS = {
    "NONE": DecodedProtocol.DECODED_PROTOCOL_NONE,
    "CSV": DecodedProtocol.DECODED_PROTOCOL_CSV,
    "MAVLINK": DecodedProtocol.DECODED_PROTOCOL_MAVLINK,
}

StatsProtocol = soldered_adsb_receiver_ns.enum("StatsProtocol")
STATS_PROTOCOL_OPTIONS = {
    "NONE": StatsProtocol.STATS_PROTOCOL_NONE,
    "CSV": StatsProtocol.STATS_PROTOCOL_CSV,
}

IncMode = soldered_adsb_receiver_ns.enum("IncMode")
REPORT_MODE_OPTIONS = {
    "ALWAYS_ONCE_PER_SECOND": IncMode.INC_ALWAYS_ONCE_PER_SECOND,
    "ONCE_PER_SECOND_IF_UPDATED": IncMode.INC_ONCE_PER_SECOND_IF_UPDATED,
    "IMMEDIATE_ON_POSITION_UPDATE": IncMode.INC_IMMEDIATE_ON_POSITION_UPDATE,
}

Baudrate = soldered_adsb_receiver_ns.enum("Baudrate")
BAUDRATE_OPTIONS = {
    57600: Baudrate.BAUDRATE_57600,
    115200: Baudrate.BAUDRATE_115200,
    921600: Baudrate.BAUDRATE_921600,
    3000000: Baudrate.BAUDRATE_3000000,
}

# Settings written to the module at boot, only the ones given in the config
BOOT_SETTINGS = (
    CONF_DECODED_PROTOCOL,
    CONF_REPORT_MODE,
    CONF_ADSB_STATISTICS,
    CONF_SYSTEM_STATISTICS,
    CONF_SYSTEM_LOG,
    CONF_BAUD_RATE,
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SolderedADSBReceiver),
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DECODED_PROTOCOL): cv.enum(
                DECODED_PROTOCOL_OPTIONS, upper=True
            ),
            cv.Optional(CONF_REPORT_MODE): cv.enum(
                REPORT_MODE_OPTIONS, upper=True, space="_"
            ),
            cv.Optional(CONF_ADSB_STATISTICS): cv.enum(
                STATS_PROTOCOL_OPTIONS, upper=True
            ),
            cv.Optional(CONF_SYSTEM_STATISTICS): cv.enum(
                STATS_PROTOCOL_OPTIONS, upper=True
            ),
            cv.Optional(CONF_SYSTEM_LOG): cv.boolean,
            cv.Optional(CONF_BAUD_RATE): cv.enum(BAUDRATE_OPTIONS, int=True),
            cv.Optional(CONF_ON_AIRCRAFT): automation.validate_automation({}),
            cv.Optional(CONF_ON_ADSB_STATS): automation.validate_automation({}),
            cv.Optional(CONF_ON_SYSTEM_STATS): automation.validate_automation({}),
            cv.Optional(CONF_ON_ADSB_VEHICLE): automation.validate_automation({}),
            cv.Optional(CONF_ON_MAVLINK_FRAME): automation.validate_automation({}),
            cv.Optional(CONF_ON_LINE): automation.validate_automation({}),
            cv.Optional(CONF_ON_UNKNOWN_LINE): automation.validate_automation({}),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


def _final_validate(config):
    """Checks the UART bus can carry what the configuration asks of it."""
    # Writing settings needs a way to talk to the module, and the BAUDRATE
    # setting written has to match the baud rate the bus runs at
    writes_settings = any(key in config for key in BOOT_SETTINGS)
    return uart.final_validate_device_schema(
        "soldered_adsb_receiver",
        baud_rate=config.get(CONF_BAUD_RATE),
        require_rx=True,
        require_tx=writes_settings,
    )(config)


FINAL_VALIDATE_SCHEMA = _final_validate

_CALLBACK_AUTOMATIONS = (
    automation.CallbackAutomation(
        CONF_ON_AIRCRAFT,
        "add_on_aircraft_callback",
        [(Aircraft.operator("const").operator("ref"), "x")],
    ),
    automation.CallbackAutomation(
        CONF_ON_ADSB_STATS,
        "add_on_adsb_stats_callback",
        [(ADSBStats.operator("const").operator("ref"), "x")],
    ),
    automation.CallbackAutomation(
        CONF_ON_SYSTEM_STATS,
        "add_on_system_stats_callback",
        [(SystemStats.operator("const").operator("ref"), "x")],
    ),
    automation.CallbackAutomation(
        CONF_ON_ADSB_VEHICLE,
        "add_on_adsb_vehicle_callback",
        [(ADSBVehicle.operator("const").operator("ref"), "x")],
    ),
    automation.CallbackAutomation(
        CONF_ON_MAVLINK_FRAME,
        "add_on_mavlink_frame_callback",
        [(MAVLinkFrame.operator("const").operator("ref"), "x")],
    ),
    automation.CallbackAutomation(
        CONF_ON_LINE,
        "add_on_line_callback",
        [(cg.std_string.operator("const").operator("ref"), "x")],
    ),
    automation.CallbackAutomation(
        CONF_ON_UNKNOWN_LINE,
        "add_on_unknown_line_callback",
        [(cg.std_string.operator("const").operator("ref"), "x")],
    ),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if reset_pin_config := config.get(CONF_RESET_PIN):
        reset_pin = await cg.gpio_pin_expression(reset_pin_config)
        cg.add(var.set_reset_pin(reset_pin))

    if CONF_DECODED_PROTOCOL in config:
        cg.add(var.set_boot_decoded_protocol(config[CONF_DECODED_PROTOCOL]))
    if CONF_REPORT_MODE in config:
        cg.add(var.set_boot_inc_mode(config[CONF_REPORT_MODE]))
    if CONF_ADSB_STATISTICS in config:
        cg.add(var.set_boot_adsb_statistics(config[CONF_ADSB_STATISTICS]))
    if CONF_SYSTEM_STATISTICS in config:
        cg.add(var.set_boot_system_statistics(config[CONF_SYSTEM_STATISTICS]))
    if CONF_SYSTEM_LOG in config:
        cg.add(var.set_boot_system_log(config[CONF_SYSTEM_LOG]))
    if CONF_BAUD_RATE in config:
        cg.add(var.set_boot_baudrate(config[CONF_BAUD_RATE]))

    await automation.build_callback_automations(var, config, _CALLBACK_AUTOMATIONS)
