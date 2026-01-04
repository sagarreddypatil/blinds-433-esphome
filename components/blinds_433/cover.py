import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover
from esphome.const import CONF_ID

from . import blinds_433_ns, Blinds433Hub, CONF_BLINDS_433_ID

DEPENDENCIES = ["blinds_433"]

CONF_REMOTE_ID = "remote_id"
CONF_BLIND_ID = "blind_id"

Blinds433Cover = blinds_433_ns.class_("Blinds433Cover", cover.Cover, cg.Component)

CONFIG_SCHEMA = (
    cover.cover_schema(Blinds433Cover)
    .extend(
        {
            cv.GenerateID(CONF_BLINDS_433_ID): cv.use_id(Blinds433Hub),
            cv.Required(CONF_REMOTE_ID): cv.hex_uint32_t,
            cv.Required(CONF_BLIND_ID): cv.int_range(min=0, max=15),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)

    hub = await cg.get_variable(config[CONF_BLINDS_433_ID])
    cg.add(var.set_hub(hub))
    cg.add(var.set_remote_id(config[CONF_REMOTE_ID]))
    cg.add(var.set_blind_id(config[CONF_BLIND_ID]))
