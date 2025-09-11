#pragma once

#include "esphome/core/component.h"
#include "esphome/components/lpuyat/lpuyat.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace lpuyat {

class LPUyatTextSensor : public text_sensor::TextSensor, public Component {
 public:
  void setup() override;
  void dump_config() override;
  void set_sensor_id(uint8_t sensor_id) { this->sensor_id_ = sensor_id; }

  void set_lpuyat_parent(LPUyat *parent) { this->parent_ = parent; }

 protected:
  LPUyat *parent_;
  uint8_t sensor_id_{0};
};

}  // namespace lpuyat
}  // namespace esphome
