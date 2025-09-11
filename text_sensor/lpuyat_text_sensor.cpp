#include "lpuyat_text_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace lpuyat {

static const char *const TAG = "lpuyat.text_sensor";

void LPUyatTextSensor::setup() {
  this->parent_->register_listener(this->sensor_id_, [this](const LPUyatDatapoint &datapoint) {
    switch (datapoint.type) {
      case LPUyatDatapointType::STRING:
        ESP_LOGD(TAG, "MCU reported text sensor %u is: %s", datapoint.id, datapoint.value_string.c_str());
        this->publish_state(datapoint.value_string);
        break;
      case LPUyatDatapointType::RAW: {
        std::string data = format_hex_pretty(datapoint.value_raw);
        ESP_LOGD(TAG, "MCU reported text sensor %u is: %s", datapoint.id, data.c_str());
        this->publish_state(data);
        break;
      }
      case LPUyatDatapointType::ENUM: {
        std::string data = to_string(datapoint.value_enum);
        ESP_LOGD(TAG, "MCU reported text sensor %u is: %s", datapoint.id, data.c_str());
        this->publish_state(data);
        break;
      }
      default:
        ESP_LOGW(TAG, "Unsupported data type for lpuyat text sensor %u: %#02hhX", datapoint.id, (uint8_t) datapoint.type);
        break;
    }
  });
}

void LPUyatTextSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "LPUyat Text Sensor:");
  ESP_LOGCONFIG(TAG, "  Text Sensor has datapoint ID %u", this->sensor_id_);
}

}  // namespace lpuyat
}  // namespace esphome
