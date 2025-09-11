#pragma once

#include "esphome/core/component.h"
#include "esphome/components/lpuyat/lpuyat.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace lpuyat {

class LPUyatSwitch : public switch_::Switch, public Component {
 public:
  void setup() override;
  void dump_config() override;
  void set_switch_id(uint8_t switch_id) { this->switch_id_ = switch_id; }

  void set_lpuyat_parent(LPUyat *parent) { this->parent_ = parent; }

 protected:
  void write_state(bool state) override;

  LPUyat *parent_;
  uint8_t switch_id_{0};
};

}  // namespace lpuyat
}  // namespace esphome
