import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_ID, CONF_PIN

CODEOWNERS = ["@you"]
MULTI_CONF = False

CONF_BLINDS_433_ID = "blinds_433_id"

blinds_433_ns = cg.esphome_ns.namespace("blinds_433")
Blinds433Hub = blinds_433_ns.class_("Blinds433Hub", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Blinds433Hub),
        cv.Required(CONF_PIN): pins.gpio_output_pin_schema,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))
