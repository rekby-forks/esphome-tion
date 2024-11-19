#include "esphome/core/log.h"
#include "esphome/core/defines.h"

#include "tion_lt_uart_vport.h"

namespace esphome {
namespace tion {

static const char *const TAG = "tion_lt_uart_vport";

void TionLtUartVPort::dump_config() { VPORT_UART_LOG("Tion LT UART"); }

}  // namespace tion
}  // namespace esphome
