#pragma once

#include <cinttypes>
#include <vector>
#include <deque>

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/time.h"
#endif

namespace esphome {
namespace lpuyat {

enum LPUyatNetworkStatus: uint8_t {
  SMARTCONFIG = 0x00,
  AP_MODE = 0x01,
  WIFI_CONFIGURED = 0x02,
  WIFI_CONNECTED = 0x03,
  CLOUD_CONNECTED = 0x04,
};

enum LPUyatDatapointType {
  RAW = 0x00,      // variable length
  BOOLEAN = 0x01,  // 1 byte (0/1)
  INTEGER = 0x02,  // 4 byte
  STRING = 0x03,   // variable length
  ENUM = 0x04,     // 1 byte
  BITMASK = 0x05,  // 1/2/4 bytes
};

struct LPUyatDatapoint {
  uint8_t id;
  LPUyatDatapointType type;
  size_t len;
  union {
    bool value_bool;
    int value_int;
    uint32_t value_uint;
    uint8_t value_enum;
    uint32_t value_bitmask;
  };
  std::string value_string;
  std::vector<uint8_t> value_raw;
};

struct LPUyatDatapointListener {
  uint8_t datapoint_id;
  std::function<void(LPUyatDatapoint)> on_datapoint;
};

enum class LPUyatCommandType : uint8_t {
/*
  HEARTBEAT = 0x00,   // module -> mcu
*/
  PRODUCT_QUERY = 0x01,   // module -> mcu
  WIFI_STATUS = 0x02,      // module -> mcu
  WIFI_RESET = 0x03,      // mcu -> module
  WIFI_SELECT = 0x04,     // mcu -> module
  DATAPOINT_REALTIME_REPORT = 0x05, // mcu -> module
  DATAPOINT_RECORDED_REPORT = 0x08, // mcu -> module
  SEND_MODULE_COMMAND = 0x09, // module -> mcu
  LOCAL_TIME_QUERY = 0x06, // mcu -> module
  WIFI_TEST = 0x07,       // mcu -> module
  FIRMWARE_UPGRADE_MODULE = 0x0A, // mcu -> module
  FIRMWARE_UPGRADE_MCU = 0x0C, // mcu -> module
  QUERY_SIGNAL_STRENGTH = 0x0B, // mcu -> module
  OBTAIN_DP_CACHE = 0x10, // mcu -> module
};

enum class LPUyatInitState : uint8_t {
  INIT_PRODUCT,
  INIT_WIFI,
  INIT_DATAPOINT,
  INIT_DONE,
};

struct LPUyatCommand {
  LPUyatCommandType cmd;
  std::vector<uint8_t> payload;
};

class LPUyat : public Component, public uart::UARTDevice {
 public:
  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void register_listener(uint8_t datapoint_id, const std::function<void(LPUyatDatapoint)> &func);
  void set_raw_datapoint_value(uint8_t datapoint_id, const std::vector<uint8_t> &value);
  void set_boolean_datapoint_value(uint8_t datapoint_id, bool value);
  void set_integer_datapoint_value(uint8_t datapoint_id, uint32_t value);
  void set_status_pin(InternalGPIOPin *status_pin) { this->status_pin_ = status_pin; }
  void set_string_datapoint_value(uint8_t datapoint_id, const std::string &value);
  void set_enum_datapoint_value(uint8_t datapoint_id, uint8_t value);
  void set_bitmask_datapoint_value(uint8_t datapoint_id, uint32_t value, uint8_t length);
  void force_set_raw_datapoint_value(uint8_t datapoint_id, const std::vector<uint8_t> &value);
  void force_set_boolean_datapoint_value(uint8_t datapoint_id, bool value);
  void force_set_integer_datapoint_value(uint8_t datapoint_id, uint32_t value);
  void force_set_string_datapoint_value(uint8_t datapoint_id, const std::string &value);
  void force_set_enum_datapoint_value(uint8_t datapoint_id, uint8_t value);
  void force_set_bitmask_datapoint_value(uint8_t datapoint_id, uint32_t value, uint8_t length);
  void send_generic_command(const LPUyatCommand &command) { send_command_(command); }
  LPUyatInitState get_init_state();
#ifdef USE_TIME
  void set_time_id(time::RealTimeClock *time_id) { this->time_id_ = time_id; }
#endif
  void add_ignore_mcu_update_on_datapoints(uint8_t ignore_mcu_update_on_datapoints) {
    this->ignore_mcu_update_on_datapoints_.push_back(ignore_mcu_update_on_datapoints);
  }
  void add_on_initialized_callback(std::function<void()> callback) {
    this->initialized_callback_.add(std::move(callback));
  }

  void set_dp_ack_delay(uint32_t delay_ms) { this->dp_ack_delay_ms_ = delay_ms; }

 protected:
  void handle_input_buffer_();
  void handle_datapoints_(const uint8_t *buffer, size_t len);
  optional<LPUyatDatapoint> get_datapoint_(uint8_t datapoint_id);
  // returns number of bytes to remove from the beginning of rx buffer
  std::size_t validate_message_();

  void handle_command_(uint8_t command, uint8_t version, const uint8_t *buffer, size_t len);
  void send_raw_command_(LPUyatCommand command);
  void process_command_queue_();
  void send_command_(const LPUyatCommand &command);
  void send_empty_command_(LPUyatCommandType command);
  void set_numeric_datapoint_value_(uint8_t datapoint_id, LPUyatDatapointType datapoint_type, uint32_t value,
                                    uint8_t length, bool forced);
  void set_string_datapoint_value_(uint8_t datapoint_id, const std::string &value, bool forced);
  void set_raw_datapoint_value_(uint8_t datapoint_id, const std::vector<uint8_t> &value, bool forced);
  void send_datapoint_command_(uint8_t datapoint_id, LPUyatDatapointType datapoint_type, std::vector<uint8_t> data);
  void set_status_pin_();
  void send_wifi_status_(const uint8_t status);
  void query_product_info_with_retries_();
  void send_wifi_status_with_timeout_(const uint32_t timeout);
  void send_firmware_upgrade_state_(LPUyatCommandType command, uint8_t state);

#ifdef USE_TIME
  void send_local_time_();
  time::RealTimeClock *time_id_{nullptr};
  bool time_sync_callback_registered_{false};
#endif
  LPUyatInitState init_state_ = LPUyatInitState::INIT_PRODUCT;
  bool init_failed_{false};
  int init_retries_{0};
  uint8_t protocol_version_ = 0;  // always 0 for low-power
  InternalGPIOPin *status_pin_{nullptr};
  int status_pin_reported_ = -1;
  int reset_pin_reported_ = -1;
  uint32_t last_command_timestamp_ = 0;
  uint32_t last_rx_char_timestamp_ = 0;
  std::string product_ = "";
  std::vector<LPUyatDatapointListener> listeners_;
  std::vector<LPUyatDatapoint> datapoints_;
  std::deque<uint8_t> rx_message_;
  std::vector<uint8_t> ignore_mcu_update_on_datapoints_{};
  std::vector<LPUyatCommand> command_queue_;
  optional<LPUyatCommandType> expected_response_{};
  CallbackManager<void()> initialized_callback_{};
  LPUyatNetworkStatus wifi_status_{LPUyatNetworkStatus::SMARTCONFIG};
  uint32_t dp_ack_delay_ms_ = 200;
};

}  // namespace lpuyat
}  // namespace esphome
