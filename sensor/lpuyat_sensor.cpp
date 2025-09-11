#include "esphome/core/log.h"
#include "lpuyat_sensor.h"
#include <cinttypes>

namespace esphome {
namespace lpuyat {

static const char *const TAG = "lpuyat.sensor";

void LPUyatSensor::setup() {
  this->parent_->register_listener(this->sensor_id_, [this](const LPUyatDatapoint &datapoint) {
    if (datapoint.type == LPUyatDatapointType::BOOLEAN) {
      ESP_LOGV(TAG, "MCU reported sensor %u is: %s", datapoint.id, ONOFF(datapoint.value_bool));
      this->publish_state(datapoint.value_bool);
    } else if (datapoint.type == LPUyatDatapointType::INTEGER) {
      ESP_LOGV(TAG, "MCU reported sensor %u is: %d", datapoint.id, datapoint.value_int);
      this->publish_state(datapoint.value_int);
    } else if (datapoint.type == LPUyatDatapointType::ENUM) {
      ESP_LOGV(TAG, "MCU reported sensor %u is: %u", datapoint.id, datapoint.value_enum);
      this->publish_state(datapoint.value_enum);
    } else if (datapoint.type == LPUyatDatapointType::BITMASK) {
      ESP_LOGV(TAG, "MCU reported sensor %u is: %" PRIx32, datapoint.id, datapoint.value_bitmask);
      this->publish_state(datapoint.value_bitmask);
    }
  });
}

void LPUyatSensor::dump_config() {
  LOG_SENSOR("", "LPUyat Sensor", this);
  ESP_LOGCONFIG(TAG, "  Sensor has datapoint ID %u", this->sensor_id_);
}

}  // namespace lpuyat
}  // namespace esphome
