#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "lpuyat.h"

#include <vector>

namespace esphome {
namespace lpuyat {

class LPUyatDatapointUpdateTrigger : public Trigger<LPUyatDatapoint> {
 public:
  explicit LPUyatDatapointUpdateTrigger(LPUyat *parent, uint8_t sensor_id) {
    parent->register_listener(sensor_id, [this](const LPUyatDatapoint &dp) { this->trigger(dp); });
  }
};

class LPUyatRawDatapointUpdateTrigger : public Trigger<std::vector<uint8_t>> {
 public:
  explicit LPUyatRawDatapointUpdateTrigger(LPUyat *parent, uint8_t sensor_id);
};

class LPUyatBoolDatapointUpdateTrigger : public Trigger<bool> {
 public:
  explicit LPUyatBoolDatapointUpdateTrigger(LPUyat *parent, uint8_t sensor_id);
};

class LPUyatIntDatapointUpdateTrigger : public Trigger<int> {
 public:
  explicit LPUyatIntDatapointUpdateTrigger(LPUyat *parent, uint8_t sensor_id);
};

class LPUyatUIntDatapointUpdateTrigger : public Trigger<uint32_t> {
 public:
  explicit LPUyatUIntDatapointUpdateTrigger(LPUyat *parent, uint8_t sensor_id);
};

class LPUyatStringDatapointUpdateTrigger : public Trigger<std::string> {
 public:
  explicit LPUyatStringDatapointUpdateTrigger(LPUyat *parent, uint8_t sensor_id);
};

class LPUyatEnumDatapointUpdateTrigger : public Trigger<uint8_t> {
 public:
  explicit LPUyatEnumDatapointUpdateTrigger(LPUyat *parent, uint8_t sensor_id);
};

class LPUyatBitmaskDatapointUpdateTrigger : public Trigger<uint32_t> {
 public:
  explicit LPUyatBitmaskDatapointUpdateTrigger(LPUyat *parent, uint8_t sensor_id);
};

}  // namespace lpuyat
}  // namespace esphome
