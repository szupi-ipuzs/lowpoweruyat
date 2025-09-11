#include "esphome/core/log.h"

#include "automation.h"

static const char *const TAG = "lpuyat.automation";

namespace esphome {
namespace lpuyat {

void check_expected_datapoint(const LPUyatDatapoint &dp, LPUyatDatapointType expected) {
  if (dp.type != expected) {
    ESP_LOGW(TAG, "LPUyat sensor %u expected datapoint type %#02hhX but got %#02hhX", dp.id,
             static_cast<uint8_t>(expected), static_cast<uint8_t>(dp.type));
  }
}

LPUyatRawDatapointUpdateTrigger::LPUyatRawDatapointUpdateTrigger(LPUyat *parent, uint8_t sensor_id) {
  parent->register_listener(sensor_id, [this](const LPUyatDatapoint &dp) {
    check_expected_datapoint(dp, LPUyatDatapointType::RAW);
    this->trigger(dp.value_raw);
  });
}

LPUyatBoolDatapointUpdateTrigger::LPUyatBoolDatapointUpdateTrigger(LPUyat *parent, uint8_t sensor_id) {
  parent->register_listener(sensor_id, [this](const LPUyatDatapoint &dp) {
    check_expected_datapoint(dp, LPUyatDatapointType::BOOLEAN);
    this->trigger(dp.value_bool);
  });
}

LPUyatIntDatapointUpdateTrigger::LPUyatIntDatapointUpdateTrigger(LPUyat *parent, uint8_t sensor_id) {
  parent->register_listener(sensor_id, [this](const LPUyatDatapoint &dp) {
    check_expected_datapoint(dp, LPUyatDatapointType::INTEGER);
    this->trigger(dp.value_int);
  });
}

LPUyatUIntDatapointUpdateTrigger::LPUyatUIntDatapointUpdateTrigger(LPUyat *parent, uint8_t sensor_id) {
  parent->register_listener(sensor_id, [this](const LPUyatDatapoint &dp) {
    check_expected_datapoint(dp, LPUyatDatapointType::INTEGER);
    this->trigger(dp.value_uint);
  });
}

LPUyatStringDatapointUpdateTrigger::LPUyatStringDatapointUpdateTrigger(LPUyat *parent, uint8_t sensor_id) {
  parent->register_listener(sensor_id, [this](const LPUyatDatapoint &dp) {
    check_expected_datapoint(dp, LPUyatDatapointType::STRING);
    this->trigger(dp.value_string);
  });
}

LPUyatEnumDatapointUpdateTrigger::LPUyatEnumDatapointUpdateTrigger(LPUyat *parent, uint8_t sensor_id) {
  parent->register_listener(sensor_id, [this](const LPUyatDatapoint &dp) {
    check_expected_datapoint(dp, LPUyatDatapointType::ENUM);
    this->trigger(dp.value_enum);
  });
}

LPUyatBitmaskDatapointUpdateTrigger::LPUyatBitmaskDatapointUpdateTrigger(LPUyat *parent, uint8_t sensor_id) {
  parent->register_listener(sensor_id, [this](const LPUyatDatapoint &dp) {
    check_expected_datapoint(dp, LPUyatDatapointType::BITMASK);
    this->trigger(dp.value_bitmask);
  });
}

}  // namespace lpuyat
}  // namespace esphome
