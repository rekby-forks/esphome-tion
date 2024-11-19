#pragma once

#include "esphome/core/defines.h"

#include "../tion/tion_vport_uart.h"
#include "../tion-api/tion-api-uart-lt.h"

namespace esphome {
namespace tion {

using TionLtUartIO = TionUartIO<dentra::tion_lt::TionLtUartProtocol>;

class TionLtUartVPort : public TionVPortUARTComponent<TionLtUartIO> {
 public:
  explicit TionLtUartVPort(io_type *io) : TionVPortUARTComponent(io) {}

  void dump_config() override;

  void set_api(void *) {}
};

}  // namespace tion
}  // namespace esphome
