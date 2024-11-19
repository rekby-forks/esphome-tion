import esphome.codegen as cg

# pylint: disable-next=relative-beyond-top-level
from .. import tion, vport

AUTO_LOAD = ["vport", "tion"]

TionLtUartVPort = tion.tion_ns.class_("TionLtUartVPort", cg.Component, vport.VPort)
TionLtUartIO = tion.tion_ns.class_("TionLtUartIO")

CONFIG_SCHEMA = vport.vport_uart_schema(TionLtUartVPort, TionLtUartIO)


async def to_code(config):
    await vport.setup_vport_uart(config)
