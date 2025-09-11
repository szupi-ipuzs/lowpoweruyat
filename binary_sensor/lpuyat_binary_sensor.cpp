#include "esphome/core/log.h"
#include "lpuyat_binary_sensor.h"

namespace esphome {
namespace lpuyat {

static const char *const TAG = "lpuyat.binary_sensor";

void LPUyatBinarySensor::setup() {
  this->parent_->register_listener(this->sensor_id_, [this](const LPUyatDatapoint &datapoint) {
    ESP_LOGV(TAG, "MCU reported binary sensor %u is: %s", datapoint.id, ONOFF(datapoint.value_bool));
    this->publish_state(datapoint.value_bool);
  });
}

void LPUyatBinarySensor::dump_config() {
  ESP_LOGCONFIG(TAG, "LPUyat Binary Sensor:");
  ESP_LOGCONFIG(TAG, "  Binary Sensor has datapoint ID %u", this->sensor_id_);
}

}  // namespace lpuyat
}  // namespace esphome
