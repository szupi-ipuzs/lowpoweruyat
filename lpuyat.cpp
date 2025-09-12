#include "lpuyat.h"
#include "esphome/components/network/util.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"

namespace esphome {
namespace lpuyat {

static const char *const TAG = "lpuyat";
static const int COMMAND_DELAY = 10;
static const int RECEIVE_TIMEOUT = 300;

static const uint64_t UART_MAX_POLL_TIME_MS = 50;

void LPUyat::query_product_info_with_retries_()
{
  this->cancel_timeout("product");
  if (this->init_state_ != LPUyatInitState::INIT_PRODUCT)
  {
    return;
  }

  this->send_empty_command_(LPUyatCommandType::PRODUCT_QUERY);
  this->set_timeout("product", 2000, [this] {
      ESP_LOGW(TAG, "No response to PRODUCT_QUERY, retrying...");
      this->query_product_info_with_retries_();
    });
}

void LPUyat::setup() {
  // this->set_interval("heartbeat", 10000, [this] {
  //   this->send_empty_command_(LPUyatCommandType::HEARTBEAT);
  // });

  this->query_product_info_with_retries_();
  if (this->status_pin_ != nullptr) {
    this->status_pin_->digital_write(false);
  }
}

void LPUyat::loop() {
  const auto start_ts = millis();
  uint64_t now = start_ts;
  auto number_of_bytes = this->available();
  while (number_of_bytes > 0)
  {
    uint8_t c;
    this->read_byte(&c);
    this->rx_message_.push_back(c);
    this->last_rx_char_timestamp_ = millis();
    if (now >= (start_ts + UART_MAX_POLL_TIME_MS))
    {
      break;
    }

    --number_of_bytes;
  }
  this->handle_input_buffer_();
  process_command_queue_();
}

void LPUyat::dump_config() {
  this->cancel_timeout("datapoint_dump");
  ESP_LOGCONFIG(TAG, "LPUyat:");
  if (this->init_state_ != LPUyatInitState::INIT_DONE) {
    if (this->init_failed_) {
      ESP_LOGCONFIG(TAG, "  Initialization failed. Current init_state: %u",
                    static_cast<uint8_t>(this->init_state_));
    } else {
      ESP_LOGCONFIG(TAG,
                    "  Configuration will be reported when setup is complete. "
                    "Current init_state: %u",
                    static_cast<uint8_t>(this->init_state_));
    }
    ESP_LOGCONFIG(TAG, "  If no further output is received, confirm that this "
                       "is a supported LPUyat device.");
    return;
  }
  for (auto &info : this->datapoints_) {
    if (info.type == LPUyatDatapointType::RAW) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: raw (value: %s)", info.id,
                    format_hex_pretty(info.value_raw).c_str());
    } else if (info.type == LPUyatDatapointType::BOOLEAN) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: switch (value: %s)", info.id,
                    ONOFF(info.value_bool));
    } else if (info.type == LPUyatDatapointType::INTEGER) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: int value (value: %d)", info.id,
                    info.value_int);
    } else if (info.type == LPUyatDatapointType::STRING) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: string value (value: %s)", info.id,
                    info.value_string.c_str());
    } else if (info.type == LPUyatDatapointType::ENUM) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: enum (value: %d)", info.id,
                    info.value_enum);
    } else if (info.type == LPUyatDatapointType::BITMASK) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: bitmask (value: %" PRIx32 ")",
                    info.id, info.value_bitmask);
    } else {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: unknown", info.id);
    }
  }
  if ((this->status_pin_reported_ != -1) || (this->reset_pin_reported_ != -1)) {
    ESP_LOGCONFIG(TAG, "  GPIO Configuration: status: pin %d, reset: pin %d",
                  this->status_pin_reported_, this->reset_pin_reported_);
  }
  LOG_PIN("  Status Pin: ", this->status_pin_);
  ESP_LOGCONFIG(TAG, "  Product: '%s'", this->product_.c_str());
}

std::size_t LPUyat::validate_message_() {

  const auto current_size = this->rx_message_.size();
  if (current_size < (2u + 1u + 1u + 2u + 1u)) // header + version + command + length + checksum
  {
    return 0u;  // don't remove anything yet
  }

  if (this->rx_message_.at(0u) != 0x55)
  {
    return 1u;
  }

  if (this->rx_message_.at(1u) != 0xAA)
  {
    return 1u;  // remove just the first 0x55, in case it is followed by another 0x55
  }

  const uint8_t version = this->rx_message_.at(2u);
  const uint8_t command = this->rx_message_.at(3u);
  const uint16_t length = (uint16_t(this->rx_message_.at(4u)) << 8) | (uint16_t(this->rx_message_.at(5u)));
  const auto checksum_offset = 6u + length;
  if ((checksum_offset + 1u) > current_size)  // offset of data field + length + checksum
  {
    return 0u;
  }

  // Byte 6+LEN: CHECKSUM - sum of all bytes (including header) modulo 256
  const uint8_t rx_checksum = this->rx_message_.at(checksum_offset);
  uint8_t calc_checksum = 0;
  for (std::size_t i = 0; i < checksum_offset; ++i)
    calc_checksum += this->rx_message_.at(i);

  if (rx_checksum != calc_checksum) {
    ESP_LOGW(TAG, "Received invalid message checksum %02X!=%02X",
             rx_checksum, calc_checksum);
    return 1u;
  }

  // valid message
  std::vector<uint8_t> data(this->rx_message_.begin() + 6u, this->rx_message_.begin() + checksum_offset);
  ESP_LOGV(TAG, "Received LPUyat: CMD=0x%02X VERSION=%u DATA=[%s] INIT_STATE=%u",
           command, version, format_hex_pretty(data).c_str(),
           static_cast<uint8_t>(this->init_state_));
  this->handle_command_(command, version, data.data(), data.size());

  // the whole message can now be removed
  return (checksum_offset + 1u);
}

void LPUyat::handle_input_buffer_() {
  do
  {
    auto bytes_to_remove = this->validate_message_();
    if (bytes_to_remove == 0)
    {
      break;
    }
    if (bytes_to_remove > this->rx_message_.size()) // just for safety, in case the validate_message_() is buggy
    {
      ESP_LOGW(TAG, "BUG: tryng to remove more bytes than possible %zu > %zu",
        bytes_to_remove, this->rx_message_.size());
      bytes_to_remove = this->rx_message_.size();
    }

    this->rx_message_.erase(this->rx_message_.begin(), this->rx_message_.begin() + bytes_to_remove);
  } while ((this->command_queue_.empty()) && (!this->rx_message_.empty()));  // stop if there's message to be sent or no input
}

void LPUyat::handle_command_(uint8_t command, uint8_t version,
                           const uint8_t *buffer, size_t len) {
  LPUyatCommandType command_type = (LPUyatCommandType)command;

  if (this->expected_response_.has_value() &&
      this->expected_response_ == command_type) {
    this->expected_response_.reset();
    this->command_queue_.erase(command_queue_.begin());
    this->init_retries_ = 0;
  }

  switch (command_type) {
  case LPUyatCommandType::PRODUCT_QUERY: {
    this->cancel_timeout("product");
    // check it is a valid string made up of printable characters
    bool valid = true;
    for (size_t i = 0; i < len; i++) {
      if (!std::isprint(buffer[i])) {
        valid = false;
        break;
      }
    }
    if (valid) {
      this->product_ = std::string(reinterpret_cast<const char *>(buffer), len);
    } else {
      this->product_ = R"({"p":"INVALID"})";
    }
    if (this->init_state_ == LPUyatInitState::INIT_PRODUCT) {
      this->init_state_ = LPUyatInitState::INIT_WIFI;
      this->wifi_status_ = LPUyatNetworkStatus::WIFI_CONFIGURED;
      this->send_wifi_status_with_timeout_(100);
    }
    break;
  }
  case LPUyatCommandType::WIFI_STATUS:
    if (this->init_state_ == LPUyatInitState::INIT_WIFI) {
      if ((this->wifi_status_ == LPUyatNetworkStatus::SMARTCONFIG) || (this->wifi_status_ == LPUyatNetworkStatus::AP_MODE))
      {
        this->wifi_status_ = LPUyatNetworkStatus::WIFI_CONFIGURED;
        this->send_wifi_status_with_timeout_(100);
      }
      else if (this->wifi_status_ == LPUyatNetworkStatus::WIFI_CONFIGURED)
      {
        this->set_interval("wifi_status", 100, [this] {
          if (LPUyatInitState::INIT_WIFI == this->init_state_)
          {
            if (esphome::network::is_connected())
            {
              this->wifi_status_ = LPUyatNetworkStatus::WIFI_CONNECTED;
              this->send_wifi_status_(static_cast<uint8_t>(this->wifi_status_));
              this->cancel_interval("wifi_status");
            }
          }
          else
          {
            this->cancel_interval("wifi_status");
          }
        });
      }
      else if (this->wifi_status_ == LPUyatNetworkStatus::WIFI_CONNECTED)
      {
        this->set_interval("wifi_status", 100, [this] {
          if (LPUyatInitState::INIT_WIFI == this->init_state_)
          {
            if (esphome::remote_is_connected())
            {
              this->wifi_status_ = LPUyatNetworkStatus::CLOUD_CONNECTED;
              this->send_wifi_status_(static_cast<uint8_t>(this->wifi_status_));
              this->cancel_interval("wifi_status");
            }
          }
          else
          {
            this->cancel_interval("wifi_status");
          }
        });
      }
      else if (this->wifi_status_ == LPUyatNetworkStatus::CLOUD_CONNECTED)
      {
        this->init_state_ = LPUyatInitState::INIT_DATAPOINT;
        this->set_timeout("datapoint_dump", 1000,
                          [this] { this->dump_config(); });
        this->initialized_callback_.call();
      }
    }
    break;
  case LPUyatCommandType::WIFI_RESET:
    ESP_LOGE(TAG, "WIFI_RESET");
    this->send_empty_command_(LPUyatCommandType::WIFI_RESET);
    this->init_state_ = LPUyatInitState::INIT_WIFI;
    this->wifi_status_ = LPUyatNetworkStatus::WIFI_CONFIGURED;
    this->send_wifi_status_with_timeout_(100);
    break;
  case LPUyatCommandType::WIFI_SELECT:
    ESP_LOGE(TAG, "WIFI_SELECT");
    this->send_empty_command_(LPUyatCommandType::WIFI_SELECT);
    this->init_state_ = LPUyatInitState::INIT_WIFI;
    this->wifi_status_ = LPUyatNetworkStatus::WIFI_CONFIGURED;
    if (len > 0)
    {
      if (buffer[0] == 0x00)
      {
        this->wifi_status_ = LPUyatNetworkStatus::SMARTCONFIG;
      }
      else if (buffer[0] == 0x01)
      {
        this->wifi_status_ = LPUyatNetworkStatus::AP_MODE;
      }
    }
    this->send_wifi_status_with_timeout_(100);
    break;
  case LPUyatCommandType::QUERY_SIGNAL_STRENGTH:
    this->send_command_(
        LPUyatCommand{.cmd = LPUyatCommandType::QUERY_SIGNAL_STRENGTH,
                    .payload = std::vector<uint8_t>{0x01, 100u}});  // fake RSSI for now
    break;
  case LPUyatCommandType::DATAPOINT_REALTIME_REPORT:
    if (this->init_state_ == LPUyatInitState::INIT_DATAPOINT) {
      this->init_state_ = LPUyatInitState::INIT_DONE;
      this->set_timeout("datapoint_dump", 1000,
                        [this] { this->dump_config(); });
      this->initialized_callback_.call();
    }
    this->handle_datapoints_(buffer, len);
    // delay the response to increase chances of publishing the values before mcu cuts the power
    this->set_timeout("datapoint_ack", 200, [this] {
        this->send_command_(
            LPUyatCommand{.cmd = LPUyatCommandType::DATAPOINT_REALTIME_REPORT,
                        .payload = std::vector<uint8_t>{0x00}});
      });
    break;

  case LPUyatCommandType::DATAPOINT_RECORDED_REPORT:
  {
    if (len <= 7)
    {
      break;
    }

    // skip time
    buffer += 7;
    len -= 7;

    if (this->init_state_ == LPUyatInitState::INIT_DATAPOINT) {
      this->init_state_ = LPUyatInitState::INIT_DONE;
      this->set_timeout("datapoint_dump", 1000,
                        [this] { this->dump_config(); });
      this->initialized_callback_.call();
    }
    this->handle_datapoints_(buffer, len);
    // delay the response to increase chances of publishing the values before mcu cuts the power
    this->set_timeout("datapoint_ack", 200, [this] {
        this->send_command_(
            LPUyatCommand{.cmd = LPUyatCommandType::DATAPOINT_RECORDED_REPORT,
                        .payload = std::vector<uint8_t>{0x00}});
        });
    break;
  }
  case LPUyatCommandType::WIFI_TEST:
    this->send_command_(
        LPUyatCommand{.cmd = LPUyatCommandType::WIFI_TEST,
                    .payload = std::vector<uint8_t>{0x00, 0x00}});
    break;
  case LPUyatCommandType::LOCAL_TIME_QUERY:
#ifdef USE_TIME
    if (this->time_id_ != nullptr) {
      this->send_local_time_();

      if (!this->time_sync_callback_registered_) {
        // lpuyat mcu supports time, so we let them know when our time changed
        this->time_id_->add_on_time_sync_callback(
            [this] { this->send_local_time_(); });
        this->time_sync_callback_registered_ = true;
      }
    } else
#endif
    {
      ESP_LOGW(
          TAG,
          "LOCAL_TIME_QUERY is not handled because time is not configured");
    }
    break;
  case LPUyatCommandType::SEND_MODULE_COMMAND:  // ack only
    break;
  case LPUyatCommandType::FIRMWARE_UPGRADE_MODULE:
  {
    this->send_firmware_upgrade_state_(LPUyatCommandType::FIRMWARE_UPGRADE_MODULE, 0x00); // means "(start to detect firmware upgrading) do not power off"
    this->set_timeout("firmware_upgrade_module", 100, [this] {
        this->cancel_timeout("firmware_upgrade_module");
        this->send_firmware_upgrade_state_(LPUyatCommandType::FIRMWARE_UPGRADE_MODULE, 0x01); // means "(the latest firmware already) power off"
      });
    break;
  }
  case LPUyatCommandType::FIRMWARE_UPGRADE_MCU:
  {
    this->send_firmware_upgrade_state_(LPUyatCommandType::FIRMWARE_UPGRADE_MCU, 0x00); // means "(start to detect firmware upgrading) do not power off"
    this->set_timeout("firmware_upgrade_mcu", 100, [this] {
        this->cancel_timeout("firmware_upgrade_mcu");
        this->send_firmware_upgrade_state_(LPUyatCommandType::FIRMWARE_UPGRADE_MCU, 0x01); // means "(the latest firmware already) power off"
      });
    break;
  }
  case LPUyatCommandType::OBTAIN_DP_CACHE:
    this->send_command_(
        LPUyatCommand{.cmd = LPUyatCommandType::OBTAIN_DP_CACHE,
                    .payload = std::vector<uint8_t>{0x01, 0x00}});  // indicate success but 0 datapoints in cache
    break;
  default:
    ESP_LOGE(TAG, "Invalid command (0x%02X) received", command);
  }
}

void LPUyat::handle_datapoints_(const uint8_t *buffer, size_t len) {
  while (len >= 4) {
    LPUyatDatapoint datapoint{};
    datapoint.id = buffer[0];
    datapoint.type = (LPUyatDatapointType)buffer[1];
    datapoint.value_uint = 0;

    size_t data_size = (buffer[2] << 8) + buffer[3];
    const uint8_t *data = buffer + 4;
    size_t data_len = len - 4;
    if (data_size > data_len) {
      ESP_LOGW(TAG,
               "Datapoint %u is truncated and cannot be parsed (%zu > %zu)",
               datapoint.id, data_size, data_len);
      return;
    }

    datapoint.len = data_size;

    switch (datapoint.type) {
    case LPUyatDatapointType::RAW:
      datapoint.value_raw = std::vector<uint8_t>(data, data + data_size);
      ESP_LOGD(TAG, "Datapoint %u update to %s", datapoint.id,
               format_hex_pretty(datapoint.value_raw).c_str());
      break;
    case LPUyatDatapointType::BOOLEAN:
      if (data_size != 1) {
        ESP_LOGW(TAG, "Datapoint %u has bad boolean len %zu", datapoint.id,
                 data_size);
        return;
      }
      datapoint.value_bool = data[0];
      ESP_LOGD(TAG, "Datapoint %u update to %s", datapoint.id,
               ONOFF(datapoint.value_bool));
      break;
    case LPUyatDatapointType::INTEGER:
      if (data_size != 4) {
        ESP_LOGW(TAG, "Datapoint %u has bad integer len %zu", datapoint.id,
                 data_size);
        return;
      }
      datapoint.value_uint = encode_uint32(data[0], data[1], data[2], data[3]);
      ESP_LOGD(TAG, "Datapoint %u update to %d", datapoint.id,
               datapoint.value_int);
      break;
    case LPUyatDatapointType::STRING:
      datapoint.value_string =
          std::string(reinterpret_cast<const char *>(data), data_size);
      ESP_LOGD(TAG, "Datapoint %u update to %s", datapoint.id,
               datapoint.value_string.c_str());
      break;
    case LPUyatDatapointType::ENUM:
      if (data_size != 1) {
        ESP_LOGW(TAG, "Datapoint %u has bad enum len %zu", datapoint.id,
                 data_size);
        return;
      }
      datapoint.value_enum = data[0];
      ESP_LOGD(TAG, "Datapoint %u update to %d", datapoint.id,
               datapoint.value_enum);
      break;
    case LPUyatDatapointType::BITMASK:
      switch (data_size) {
      case 1:
        datapoint.value_bitmask = encode_uint32(0, 0, 0, data[0]);
        break;
      case 2:
        datapoint.value_bitmask = encode_uint32(0, 0, data[0], data[1]);
        break;
      case 4:
        datapoint.value_bitmask =
            encode_uint32(data[0], data[1], data[2], data[3]);
        break;
      default:
        ESP_LOGW(TAG, "Datapoint %u has bad bitmask len %zu", datapoint.id,
                 data_size);
        return;
      }
      ESP_LOGD(TAG, "Datapoint %u update to %#08" PRIX32, datapoint.id,
               datapoint.value_bitmask);
      break;
    default:
      ESP_LOGW(TAG, "Datapoint %u has unknown type %#02hhX", datapoint.id,
               static_cast<uint8_t>(datapoint.type));
      return;
    }

    len -= data_size + 4;
    buffer = data + data_size;

    // drop update if datapoint is in ignore_mcu_datapoint_update list
    bool skip = false;
    for (auto i : this->ignore_mcu_update_on_datapoints_) {
      if (datapoint.id == i) {
        ESP_LOGV(TAG,
                 "Datapoint %u found in ignore_mcu_update_on_datapoints list, "
                 "dropping MCU update",
                 datapoint.id);
        skip = true;
        break;
      }
    }
    if (skip)
      continue;

    // Update internal datapoints
    bool found = false;
    for (auto &other : this->datapoints_) {
      if (other.id == datapoint.id) {
        other = datapoint;
        found = true;
      }
    }
    if (!found) {
      this->datapoints_.push_back(datapoint);
    }

    // Run through listeners
    for (auto &listener : this->listeners_) {
      if (listener.datapoint_id == datapoint.id)
        listener.on_datapoint(datapoint);
    }
  }
}

void LPUyat::send_raw_command_(LPUyatCommand command) {
  uint8_t len_hi = (uint8_t)(command.payload.size() >> 8);
  uint8_t len_lo = (uint8_t)(command.payload.size() & 0xFF);
  uint8_t version = 0;

  this->last_command_timestamp_ = millis();
  switch (command.cmd) {
  case LPUyatCommandType::PRODUCT_QUERY:
    this->expected_response_ = LPUyatCommandType::PRODUCT_QUERY;
    break;
  case LPUyatCommandType::WIFI_STATUS:
    this->expected_response_ = LPUyatCommandType::WIFI_STATUS;
    break;
  default:
    break;
  }

  ESP_LOGV(TAG, "Sending LPUyat: CMD=0x%02X VERSION=%u DATA=[%s] INIT_STATE=%u",
           static_cast<uint8_t>(command.cmd), version,
           format_hex_pretty(command.payload).c_str(),
           static_cast<uint8_t>(this->init_state_));

  this->write_array(
      {0x55, 0xAA, version, (uint8_t)command.cmd, len_hi, len_lo});
  if (!command.payload.empty())
    this->write_array(command.payload.data(), command.payload.size());

  uint8_t checksum = 0x55 + 0xAA + (uint8_t)command.cmd + len_hi + len_lo;
  for (auto &data : command.payload)
    checksum += data;
  this->write_byte(checksum);
}

void LPUyat::process_command_queue_() {
  uint32_t now = millis();
  uint32_t delay = now - this->last_command_timestamp_;

  if (now - this->last_rx_char_timestamp_ > RECEIVE_TIMEOUT) {
    this->rx_message_.clear();
  }

  if (this->expected_response_.has_value() && delay > RECEIVE_TIMEOUT) {
    this->expected_response_.reset();
    this->command_queue_.erase(command_queue_.begin());
    if ((this->init_state_ != LPUyatInitState::INIT_DONE) && (this->init_state_ != LPUyatInitState::INIT_PRODUCT)) {
      this->init_state_ = LPUyatInitState::INIT_PRODUCT;
      this->query_product_info_with_retries_();
    }
  }

  // Left check of delay since last command in case there's ever a command sent
  // by calling send_raw_command_ directly
  if (delay > COMMAND_DELAY && !this->command_queue_.empty() &&
      this->rx_message_.empty() && !this->expected_response_.has_value()) {
    this->send_raw_command_(command_queue_.front());
    if (!this->expected_response_.has_value())
      this->command_queue_.erase(command_queue_.begin());
  }
}

void LPUyat::send_command_(const LPUyatCommand &command) {
  command_queue_.push_back(command);
  process_command_queue_();
}

void LPUyat::send_empty_command_(LPUyatCommandType command) {
  send_command_(LPUyatCommand{.cmd = command, .payload = std::vector<uint8_t>{}});
}

void LPUyat::set_status_pin_() {
  bool is_network_ready = network::is_connected() && remote_is_connected();
  this->status_pin_->digital_write(is_network_ready);
}

void LPUyat::send_wifi_status_(const uint8_t status) {
  ESP_LOGD(TAG, "Sending WiFi Status %d", status);
  this->send_command_(LPUyatCommand{.cmd = LPUyatCommandType::WIFI_STATUS,
                                  .payload = std::vector<uint8_t>{status}});
}

void LPUyat::send_wifi_status_with_timeout_(const uint32_t timeout) {
{
  this->cancel_timeout("wifi_status");
  if (LPUyatInitState::INIT_WIFI == this->init_state_)
  {
    this->set_timeout("wifi_status", timeout, [this] {
          this->send_wifi_status_(static_cast<uint8_t>(this->wifi_status_));
        });
    }
  }
}

void LPUyat::send_firmware_upgrade_state_(LPUyatCommandType command, uint8_t state) {
  this->send_command_(LPUyatCommand{.cmd = command,
                                  .payload = std::vector<uint8_t>{state}});
}

#ifdef USE_TIME
void LPUyat::send_local_time_() {
  std::vector<uint8_t> payload;
  ESPTime now = this->time_id_->now();
  if (now.is_valid()) {
    uint8_t year = now.year - 2000;
    uint8_t month = now.month;
    uint8_t day_of_month = now.day_of_month;
    uint8_t hour = now.hour;
    uint8_t minute = now.minute;
    uint8_t second = now.second;
    // LPUyat days starts from Monday, esphome uses Sunday as day 1
    uint8_t day_of_week = now.day_of_week - 1;
    if (day_of_week == 0) {
      day_of_week = 7;
    }
    ESP_LOGD(TAG, "Sending local time");
    payload = std::vector<uint8_t>{0x01, year,   month,  day_of_month,
                                   hour, minute, second, day_of_week};
  } else {
    // By spec we need to notify MCU that the time was not obtained if this is a
    // response to a query
    ESP_LOGW(TAG, "Sending missing local time");
    payload =
        std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  }
  this->send_command_(LPUyatCommand{.cmd = LPUyatCommandType::LOCAL_TIME_QUERY,
                                  .payload = payload});
}
#endif

void LPUyat::set_raw_datapoint_value(uint8_t datapoint_id,
                                   const std::vector<uint8_t> &value) {
  this->set_raw_datapoint_value_(datapoint_id, value, false);
}

void LPUyat::set_boolean_datapoint_value(uint8_t datapoint_id, bool value) {
  this->set_numeric_datapoint_value_(datapoint_id, LPUyatDatapointType::BOOLEAN,
                                     value, 1, false);
}

void LPUyat::set_integer_datapoint_value(uint8_t datapoint_id, uint32_t value) {
  this->set_numeric_datapoint_value_(datapoint_id, LPUyatDatapointType::INTEGER,
                                     value, 4, false);
}

void LPUyat::set_string_datapoint_value(uint8_t datapoint_id,
                                      const std::string &value) {
  this->set_string_datapoint_value_(datapoint_id, value, false);
}

void LPUyat::set_enum_datapoint_value(uint8_t datapoint_id, uint8_t value) {
  this->set_numeric_datapoint_value_(datapoint_id, LPUyatDatapointType::ENUM,
                                     value, 1, false);
}

void LPUyat::set_bitmask_datapoint_value(uint8_t datapoint_id, uint32_t value,
                                       uint8_t length) {
  this->set_numeric_datapoint_value_(datapoint_id, LPUyatDatapointType::BITMASK,
                                     value, length, false);
}

void LPUyat::force_set_raw_datapoint_value(uint8_t datapoint_id,
                                         const std::vector<uint8_t> &value) {
  this->set_raw_datapoint_value_(datapoint_id, value, true);
}

void LPUyat::force_set_boolean_datapoint_value(uint8_t datapoint_id, bool value) {
  this->set_numeric_datapoint_value_(datapoint_id, LPUyatDatapointType::BOOLEAN,
                                     value, 1, true);
}

void LPUyat::force_set_integer_datapoint_value(uint8_t datapoint_id,
                                             uint32_t value) {
  this->set_numeric_datapoint_value_(datapoint_id, LPUyatDatapointType::INTEGER,
                                     value, 4, true);
}

void LPUyat::force_set_string_datapoint_value(uint8_t datapoint_id,
                                            const std::string &value) {
  this->set_string_datapoint_value_(datapoint_id, value, true);
}

void LPUyat::force_set_enum_datapoint_value(uint8_t datapoint_id, uint8_t value) {
  this->set_numeric_datapoint_value_(datapoint_id, LPUyatDatapointType::ENUM,
                                     value, 1, true);
}

void LPUyat::force_set_bitmask_datapoint_value(uint8_t datapoint_id,
                                             uint32_t value, uint8_t length) {
  this->set_numeric_datapoint_value_(datapoint_id, LPUyatDatapointType::BITMASK,
                                     value, length, true);
}

optional<LPUyatDatapoint> LPUyat::get_datapoint_(uint8_t datapoint_id) {
  for (auto &datapoint : this->datapoints_) {
    if (datapoint.id == datapoint_id)
      return datapoint;
  }
  return {};
}

void LPUyat::set_numeric_datapoint_value_(uint8_t datapoint_id,
                                        LPUyatDatapointType datapoint_type,
                                        const uint32_t value, uint8_t length,
                                        bool forced) {
  ESP_LOGD(TAG, "Setting datapoint %u to %" PRIu32, datapoint_id, value);
  optional<LPUyatDatapoint> datapoint = this->get_datapoint_(datapoint_id);
  if (!datapoint.has_value()) {
    ESP_LOGW(TAG, "Setting unknown datapoint %u", datapoint_id);
  } else if (datapoint->type != datapoint_type) {
    ESP_LOGE(TAG, "Attempt to set datapoint %u with incorrect type",
             datapoint_id);
    return;
  } else if (!forced && datapoint->value_uint == value) {
    ESP_LOGV(TAG, "Not sending unchanged value");
    return;
  }

  std::vector<uint8_t> data;
  switch (length) {
  case 4:
    data.push_back(value >> 24);
    data.push_back(value >> 16);
  case 2:
    data.push_back(value >> 8);
  case 1:
    data.push_back(value >> 0);
    break;
  default:
    ESP_LOGE(TAG, "Unexpected datapoint length %u", length);
    return;
  }
  this->send_datapoint_command_(datapoint_id, datapoint_type, data);
}

void LPUyat::set_raw_datapoint_value_(uint8_t datapoint_id,
                                    const std::vector<uint8_t> &value,
                                    bool forced) {
  ESP_LOGD(TAG, "Setting datapoint %u to %s", datapoint_id,
           format_hex_pretty(value).c_str());
  optional<LPUyatDatapoint> datapoint = this->get_datapoint_(datapoint_id);
  if (!datapoint.has_value()) {
    ESP_LOGW(TAG, "Setting unknown datapoint %u", datapoint_id);
  } else if (datapoint->type != LPUyatDatapointType::RAW) {
    ESP_LOGE(TAG, "Attempt to set datapoint %u with incorrect type",
             datapoint_id);
    return;
  } else if (!forced && datapoint->value_raw == value) {
    ESP_LOGV(TAG, "Not sending unchanged value");
    return;
  }
  this->send_datapoint_command_(datapoint_id, LPUyatDatapointType::RAW, value);
}

void LPUyat::set_string_datapoint_value_(uint8_t datapoint_id,
                                       const std::string &value, bool forced) {
  ESP_LOGD(TAG, "Setting datapoint %u to %s", datapoint_id, value.c_str());
  optional<LPUyatDatapoint> datapoint = this->get_datapoint_(datapoint_id);
  if (!datapoint.has_value()) {
    ESP_LOGW(TAG, "Setting unknown datapoint %u", datapoint_id);
  } else if (datapoint->type != LPUyatDatapointType::STRING) {
    ESP_LOGE(TAG, "Attempt to set datapoint %u with incorrect type",
             datapoint_id);
    return;
  } else if (!forced && datapoint->value_string == value) {
    ESP_LOGV(TAG, "Not sending unchanged value");
    return;
  }
  std::vector<uint8_t> data;
  for (char const &c : value) {
    data.push_back(c);
  }
  this->send_datapoint_command_(datapoint_id, LPUyatDatapointType::STRING, data);
}

void LPUyat::send_datapoint_command_(uint8_t datapoint_id,
                                   LPUyatDatapointType datapoint_type,
                                   std::vector<uint8_t> data) {
  std::vector<uint8_t> buffer;
  buffer.push_back(datapoint_id);
  buffer.push_back(static_cast<uint8_t>(datapoint_type));
  buffer.push_back(data.size() >> 8);
  buffer.push_back(data.size() >> 0);
  buffer.insert(buffer.end(), data.begin(), data.end());

  this->send_command_(LPUyatCommand{.cmd = LPUyatCommandType::SEND_MODULE_COMMAND,
                                  .payload = buffer});
  this->expected_response_ = LPUyatCommandType::SEND_MODULE_COMMAND;
}

void LPUyat::register_listener(uint8_t datapoint_id,
                             const std::function<void(LPUyatDatapoint)> &func) {
  auto listener = LPUyatDatapointListener{
      .datapoint_id = datapoint_id,
      .on_datapoint = func,
  };
  this->listeners_.push_back(listener);

  // Run through existing datapoints
  for (auto &datapoint : this->datapoints_) {
    if (datapoint.id == datapoint_id)
      func(datapoint);
  }
}

LPUyatInitState LPUyat::get_init_state() { return this->init_state_; }

} // namespace lpuyat
} // namespace esphome
