/**
 * @file soldered_adsb_receiver.cpp
 * @brief Implementation for the soldered_adsb_receiver ESPHome component
 *
 * The AT command set and the RUN / CONFIGURATION / BOOTLOADER state machine
 * follow the Aerobits OEM SC1/SC1-EXT datasheet.
 *
 * @author Soldered Electronics
 */

#include "soldered_adsb_receiver.h"

#include <algorithm>
#include <cctype>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::soldered_adsb_receiver {

static const char *const TAG = "soldered_adsb_receiver";

// How much is pulled off the UART per read while draining the receive buffer
static const size_t READ_CHUNK_SIZE = 64;

// Timeouts the module is given to answer, in ms
static const uint32_t AT_TIMEOUT_MS = 500;
static const uint32_t STATE_CHANGE_TIMEOUT_MS = 1500;
static const uint32_t REBOOT_TIMEOUT_MS = 2000;

// Multi-line replies have no terminal line, so they are taken as done once the
// module has been quiet for this long, in ms
static const uint32_t MULTILINE_QUIET_MS = 150;
static const uint32_t MULTILINE_TIMEOUT_MS = 1500;
static const uint32_t DESCRIPTION_TIMEOUT_MS = 800;

// Time the module is given to finish switching state and baud rate, in ms
static const uint32_t STATE_SETTLE_MS = 100;

// Longest reply kept, AT+HELP and AT+SETTINGS? run to a few kB
static const size_t MAX_REPLY_LEN = 8192;

// Every RUN baud rate the module supports, in the order they are tried when looking for it
static const uint32_t SUPPORTED_BAUD_RATES[] = {115200, 921600, 3000000, 57600};

// *****************************************************************************
// Section: Small helpers

/**
 * @brief Check whether a string starts with a prefix
 *
 * @param[in] s String
 * @param[in] prefix Prefix
 *
 * @return true if `s` starts with `prefix`
 */
static bool starts_with(const char *s, const char *prefix) { return strncmp(s, prefix, strlen(prefix)) == 0; }

/**
 * @brief Take the whitespace off both ends of a string, in place
 *
 * @param[in,out] s String
 *
 * @return Start of the trimmed string, inside `s`
 */
static char *trim_in_place(char *s) {
  while (*s != '\0' && isspace((unsigned char) *s))
    s++;

  size_t length = strlen(s);
  while (length > 0 && isspace((unsigned char) s[length - 1]))
    s[--length] = '\0';

  return s;
}

/**
 * @brief Find the value of `AT+<name>=<value>` in a reply
 *
 * @param[in] reply Reply lines, separated by '\n'
 * @param[in] name Setting name
 * @param[out] value Value found
 *
 * @return true if the reply holds the setting
 */
static bool extract_assignment(const std::string &reply, const std::string &name, std::string &value) {
  const std::string prefix = "AT+" + name + "=";

  size_t start = 0;
  while (start < reply.size()) {
    size_t end = reply.find('\n', start);
    if (end == std::string::npos)
      end = reply.size();

    if (reply.compare(start, prefix.size(), prefix) == 0 && end - start >= prefix.size()) {
      value = reply.substr(start + prefix.size(), end - start - prefix.size());
      return true;
    }

    start = end + 1;
  }

  return false;
}

/**
 * @brief Check whether a line ends the wait for an AT reply
 *
 * AT+RUN_END and AT+CONFIG_END are exit notices the module sends before the
 * real arrival marker, RUN_END before CONFIG_START and CONFIG_END before
 * RUN_START, so they must not end the wait on their own.
 *
 * @param[in] line Received line
 *
 * @return true if the line is terminal
 */
static bool is_terminal_line(const char *line) {
  return starts_with(line, "AT+OK") || starts_with(line, "AT+ERROR") || starts_with(line, "AT+CONFIG_START") ||
         starts_with(line, "AT+RUN_START") || starts_with(line, "AT+BOOTLOADER_START");
}

/// Name of an ADSB_RX_PROTOCOL_DECODED value, as the module takes it
static const char *decoded_protocol_to_string(DecodedProtocol protocol) {
  switch (protocol) {
    case DECODED_PROTOCOL_NONE:
      return "None";
    case DECODED_PROTOCOL_CSV:
      return "CSV";
    case DECODED_PROTOCOL_MAVLINK:
      return "Mavlink";
    default:
      return nullptr;
  }
}

/// Name of an ADSB_RX_PROTOCOL_INC value, for the log
static const char *inc_mode_to_string(IncMode mode) {
  switch (mode) {
    case INC_ALWAYS_ONCE_PER_SECOND:
      return "always once per second";
    case INC_ONCE_PER_SECOND_IF_UPDATED:
      return "once per second if updated";
    case INC_IMMEDIATE_ON_POSITION_UPDATE:
      return "immediate on position update";
    default:
      return "unknown";
  }
}

/// Name of a statistics protocol, for the log
static const char *stats_protocol_to_string(StatsProtocol protocol) {
  return protocol == STATS_PROTOCOL_CSV ? "CSV" : "None";
}

// *****************************************************************************
// Section: Component

void SolderedADSBReceiver::setup() {
  // Left as a high impedance input rather than driven: holding a guessed idle
  // level would keep the module in reset for good if the guess were wrong.
  // Only hardware_reset() drives it, and only for the pulse.
  if (this->reset_pin_ != nullptr)
    this->reset_pin_->pin_mode(gpio::FLAG_INPUT);

  // The `uart:` baud rate is the RUN baud rate the module is expected at
  this->run_baud_rate_ = this->parent_->get_baud_rate();

  // Whatever arrived before now is a partial line
  this->discard_input_();

  this->apply_boot_settings_();
}

void SolderedADSBReceiver::loop() {
  this->read_input_();
  this->process_queue_();
}

void SolderedADSBReceiver::dump_config() {
  ESP_LOGCONFIG(TAG, "Soldered ADS-B Receiver:");
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  ESP_LOGCONFIG(TAG, "  RUN baud rate: %" PRIu32, this->run_baud_rate_);
  if (this->boot_decoded_protocol_.has_value())
    ESP_LOGCONFIG(TAG, "  Decoded protocol: %s", decoded_protocol_to_string(*this->boot_decoded_protocol_));
  if (this->boot_inc_mode_.has_value())
    ESP_LOGCONFIG(TAG, "  Report mode: %s", inc_mode_to_string(*this->boot_inc_mode_));
  if (this->boot_adsb_statistics_.has_value())
    ESP_LOGCONFIG(TAG, "  ADS-B statistics: %s", stats_protocol_to_string(*this->boot_adsb_statistics_));
  if (this->boot_system_statistics_.has_value())
    ESP_LOGCONFIG(TAG, "  System statistics: %s", stats_protocol_to_string(*this->boot_system_statistics_));
  if (this->boot_system_log_.has_value())
    ESP_LOGCONFIG(TAG, "  System log: %s", YESNO(*this->boot_system_log_));
  if (this->boot_baudrate_.has_value())
    ESP_LOGCONFIG(TAG, "  Module baud rate: %" PRIu32, baudrate_to_bps(*this->boot_baudrate_));
}

void SolderedADSBReceiver::apply_boot_settings_() {
  if (!this->boot_decoded_protocol_.has_value() && !this->boot_inc_mode_.has_value() &&
      !this->boot_adsb_statistics_.has_value() && !this->boot_system_statistics_.has_value() &&
      !this->boot_system_log_.has_value() && !this->boot_baudrate_.has_value())
    return;

  ESP_LOGD(TAG, "Writing the configured settings to the module");

  // Settings can only be written in the CONFIGURATION state
  this->enter_configuration([this](bool success) {
    if (!success) {
      ESP_LOGE(TAG, "Could not reach the module to write its settings");
      this->status_set_warning();
    }
  });

  if (this->boot_decoded_protocol_.has_value())
    this->set_adsb_rx_protocol_decoded(*this->boot_decoded_protocol_);
  if (this->boot_inc_mode_.has_value())
    this->set_adsb_rx_protocol_inc(*this->boot_inc_mode_);
  if (this->boot_adsb_statistics_.has_value())
    this->set_adsb_statistics_protocol(*this->boot_adsb_statistics_);
  if (this->boot_system_statistics_.has_value())
    this->set_system_statistics_protocol(*this->boot_system_statistics_);
  if (this->boot_system_log_.has_value())
    this->set_system_log(*this->boot_system_log_);
  if (this->boot_baudrate_.has_value())
    this->set_baudrate(*this->boot_baudrate_);

  this->exit_configuration([this](bool success) {
    if (success) {
      ESP_LOGI(TAG, "Settings written, module is back in the RUN state");
      this->status_clear_warning();
    } else {
      ESP_LOGW(TAG, "Module did not return to the RUN state");
      this->status_set_warning();
    }
  });
}

// *****************************************************************************
// Section: State machine

bool SolderedADSBReceiver::hardware_reset(uint32_t pulse_ms, bool resync_to_115200) {
  if (this->reset_pin_ == nullptr) {
    ESP_LOGW(TAG, "No reset_pin configured, cannot reset the module");
    return false;
  }

  this->queue_.push_back(make_step_(StepType::RESET_ASSERT, 0));
  this->queue_.push_back(make_step_(StepType::WAIT, pulse_ms));
  this->queue_.push_back(make_step_(StepType::RESET_RELEASE, 0));
  if (resync_to_115200)
    this->queue_.push_back(make_step_(StepType::SET_BAUD, CONFIG_BAUD_RATE));

  return true;
}

void SolderedADSBReceiver::enter_configuration(ResultCallback callback) {
  // The RUN baud rate the module is expected at goes first, then every other one it supports
  std::vector<uint32_t> baud_rates{this->run_baud_rate_};
  for (uint32_t baud_rate : SUPPORTED_BAUD_RATES) {
    if (baud_rate != this->run_baud_rate_)
      baud_rates.push_back(baud_rate);
  }

  this->queue_enter_attempt_(std::move(baud_rates), 0, std::move(callback), false);
}

void SolderedADSBReceiver::queue_enter_attempt_(std::vector<uint32_t> baud_rates, size_t index, ResultCallback callback,
                                                bool front) {
  uint32_t baud_rate = baud_rates[index];

  // In practice the AT+RUN_END / AT+CONFIG_START notices are not always sent;
  // often only a generic AT+OK comes back, likely depending on the SYSTEM_LOG
  // setting. So success is not tied to a notice: the AT+OK is accepted, the
  // baud rate switched, and the change confirmed with AT+CONFIG? instead.
  auto handler = [this, baud_rates, index, callback](bool received, const std::string &reply) {
    if (!received && index + 1 < baud_rates.size()) {
      ESP_LOGD(TAG, "No answer to AT+CONFIG=1 at %" PRIu32 " baud, trying %" PRIu32, baud_rates[index],
               baud_rates[index + 1]);
      this->queue_enter_attempt_(baud_rates, index + 1, callback, true);
      return true;
    }

    if (!received || is_error(reply)) {
      ESP_LOGW(TAG, "Could not enter the CONFIGURATION state: %s", received ? reply.c_str() : "no answer");
      this->set_uart_baud_(this->run_baud_rate_);
      if (callback)
        callback(false);
      return false;
    }

    if (baud_rates[index] != this->run_baud_rate_) {
      ESP_LOGI(TAG, "Module answered at %" PRIu32 " baud, not at the expected %" PRIu32, baud_rates[index],
               this->run_baud_rate_);
    }

    // CONFIGURATION always runs at 115200 baud
    Step steps[] = {
        make_step_(StepType::SET_BAUD, CONFIG_BAUD_RATE), make_step_(StepType::WAIT, STATE_SETTLE_MS),
        make_command_("CONFIG?", ReplyType::QUERY, AT_TIMEOUT_MS, this->make_state_check_(true, callback), true)};
    this->queue_.insert(this->queue_.begin(), std::make_move_iterator(std::begin(steps)),
                        std::make_move_iterator(std::end(steps)));
    return true;
  };

  Step steps[] = {make_step_(StepType::SET_BAUD, baud_rate),
                  make_command_("CONFIG=1", ReplyType::TERMINAL, STATE_CHANGE_TIMEOUT_MS, handler, true)};
  auto position = front ? this->queue_.begin() : this->queue_.end();
  this->queue_.insert(position, std::make_move_iterator(std::begin(steps)), std::make_move_iterator(std::end(steps)));
}

void SolderedADSBReceiver::exit_configuration(ResultCallback callback) {
  auto handler = [this, callback](bool received, const std::string &reply) {
    if (!received || is_error(reply)) {
      ESP_LOGW(TAG, "Could not leave the CONFIGURATION state: %s", received ? reply.c_str() : "no answer");
      if (callback)
        callback(false);
      return false;
    }

    // RUN runs at the BAUDRATE setting
    Step steps[] = {
        make_step_(StepType::SET_BAUD, this->run_baud_rate_), make_step_(StepType::WAIT, STATE_SETTLE_MS),
        make_command_("CONFIG?", ReplyType::QUERY, AT_TIMEOUT_MS, this->make_state_check_(false, callback))};
    this->queue_.insert(this->queue_.begin(), std::make_move_iterator(std::begin(steps)),
                        std::make_move_iterator(std::end(steps)));
    return true;
  };

  this->queue_.push_back(make_command_("CONFIG=0", ReplyType::TERMINAL, STATE_CHANGE_TIMEOUT_MS, handler));
}

SolderedADSBReceiver::StepHandler SolderedADSBReceiver::make_state_check_(bool enter, ResultCallback callback) {
  return [this, enter, callback](bool received, const std::string &reply) {
    std::string value;
    bool reached = false;
    if (received && extract_assignment(reply, "CONFIG", value)) {
      long state = strtol(value.c_str(), nullptr, 10);
      bool in_config = (state == CONFIG_STATE_CONFIG_115200 || state == CONFIG_STATE_CONFIG_SET_BAUD);
      reached = enter ? in_config : (state == CONFIG_STATE_RUN);
      if (!reached)
        ESP_LOGW(TAG, "Module reports state %ld after AT+CONFIG=%d", state, enter ? 1 : 0);
    } else {
      ESP_LOGW(TAG, "Module did not answer AT+CONFIG? after AT+CONFIG=%d", enter ? 1 : 0);
    }

    if (reached) {
      ESP_LOGD(TAG, "Module is in the %s state", enter ? "CONFIGURATION" : "RUN");
    } else if (enter) {
      // The module is not where the queued commands expect it, go back to listening for data
      this->set_uart_baud_(this->run_baud_rate_);
    }

    if (callback)
      callback(reached);
    return reached;
  };
}

void SolderedADSBReceiver::get_config_state(ValueCallback callback) { this->queue_query_("CONFIG", callback); }

void SolderedADSBReceiver::reboot_to_bootloader(ResultCallback callback) {
  auto handler = [callback](bool received, const std::string &reply) {
    bool success = received && (is_ok(reply) || reply.find("AT+BOOTLOADER_START") != std::string::npos);
    if (!success)
      ESP_LOGW(TAG, "AT+REBOOT_BOOTLOADER failed: %s", received ? reply.c_str() : "no answer");
    if (callback)
      callback(success);
    return success;
  };

  this->queue_.push_back(make_command_("REBOOT_BOOTLOADER", ReplyType::TERMINAL, REBOOT_TIMEOUT_MS, handler));
}

void SolderedADSBReceiver::reboot(ResultCallback callback) {
  this->queue_expect_ok_("REBOOT", REBOOT_TIMEOUT_MS, callback);
}

void SolderedADSBReceiver::set_lock(bool locked, ResultCallback callback) {
  this->queue_expect_ok_(locked ? "LOCK=1" : "LOCK=0", AT_TIMEOUT_MS, callback);
}

void SolderedADSBReceiver::get_lock(ValueCallback callback) { this->queue_query_("LOCK", callback); }

void SolderedADSBReceiver::is_bootloader(ValueCallback callback) { this->queue_query_("BOOT", callback); }

void SolderedADSBReceiver::queue_delay(uint32_t ms) { this->queue_.push_back(make_step_(StepType::WAIT, ms)); }

// *****************************************************************************
// Section: AT command transport

void SolderedADSBReceiver::send_at(const std::string &body, ValueCallback callback, uint32_t timeout_ms,
                                   const std::string &wait_for_prefix) {
  // A query only ever gets a single "AT+NAME=value" reply with no AT+OK after it
  ReplyType reply = ReplyType::TERMINAL;
  if (!wait_for_prefix.empty()) {
    reply = ReplyType::PREFIX;
  } else if (!body.empty() && body.back() == '?') {
    reply = ReplyType::QUERY;
  }

  Step step = make_command_(body, reply, timeout_ms, [callback](bool received, const std::string &reply) {
    if (callback)
      callback(received, reply);
    return received;
  });
  step.prefix = wait_for_prefix;
  this->queue_.push_back(std::move(step));
}

bool SolderedADSBReceiver::is_ok(const std::string &reply) { return reply.find("AT+OK") != std::string::npos; }

bool SolderedADSBReceiver::is_error(const std::string &reply, std::string *description) {
  size_t error = reply.find("AT+ERROR");
  if (error == std::string::npos)
    return false;

  if (description != nullptr) {
    description->clear();
    size_t open = reply.find('(', error);
    size_t close = reply.find(')', error);
    if (open != std::string::npos && close != std::string::npos && close > open)
      *description = reply.substr(open + 1, close - open - 1);
  }

  return true;
}

// *****************************************************************************
// Section: Generic settings and commands

void SolderedADSBReceiver::set_setting(const std::string &name, const std::string &value, ResultCallback callback) {
  this->queue_expect_ok_(name + "=" + value, AT_TIMEOUT_MS, callback);
}

void SolderedADSBReceiver::get_setting(const std::string &name, ValueCallback callback) {
  this->queue_query_(name, callback);
}

void SolderedADSBReceiver::get_setting_description(const std::string &name, ValueCallback callback) {
  this->queue_multiline_(name + "=?", DESCRIPTION_TIMEOUT_MS, callback);
}

void SolderedADSBReceiver::get_all_settings(ValueCallback callback) {
  this->queue_multiline_("SETTINGS?", MULTILINE_TIMEOUT_MS, callback);
}

void SolderedADSBReceiver::get_full_help(ValueCallback callback) {
  this->queue_multiline_("HELP", MULTILINE_TIMEOUT_MS, callback);
}

void SolderedADSBReceiver::reset_settings_to_default(ResultCallback callback) {
  this->queue_expect_ok_("SETTINGS_DEFAULT", AT_TIMEOUT_MS, callback);
}

void SolderedADSBReceiver::get_serial_number(ValueCallback callback) { this->queue_query_("SERIAL_NUMBER", callback); }

void SolderedADSBReceiver::get_firmware_version(ValueCallback callback) {
  this->queue_query_("FIRMWARE_VERSION", callback);
}

void SolderedADSBReceiver::get_device_type(ValueCallback callback) { this->queue_query_("DEVICE", callback); }

void SolderedADSBReceiver::test(ResultCallback callback) { this->queue_expect_ok_("TEST", AT_TIMEOUT_MS, callback); }

// *****************************************************************************
// Section: Typed settings

void SolderedADSBReceiver::set_baudrate(Baudrate baudrate, ResultCallback callback) {
  if (baudrate > BAUDRATE_57600) {
    ESP_LOGE(TAG, "Invalid BAUDRATE value %u", (unsigned) baudrate);
    if (callback)
      callback(false);
    return;
  }

  char value[4];
  snprintf(value, sizeof(value), "%u", (unsigned) baudrate);

  // The new baud rate takes effect on the next entry into RUN, which exit_configuration() then switches to
  this->set_setting("BAUDRATE", value, [this, baudrate, callback](bool success) {
    if (success)
      this->run_baud_rate_ = baudrate_to_bps(baudrate);
    if (callback)
      callback(success);
  });
}

void SolderedADSBReceiver::set_system_log(bool enable, ResultCallback callback) {
  this->set_setting("SYSTEM_LOG", enable ? "1" : "0", callback);
}

void SolderedADSBReceiver::set_system_statistics_protocol(StatsProtocol protocol, ResultCallback callback) {
  if (protocol > STATS_PROTOCOL_CSV) {
    ESP_LOGE(TAG, "Invalid SYSTEM_STATISTICS value %u", (unsigned) protocol);
    if (callback)
      callback(false);
    return;
  }

  // SYSTEM_STATISTICS is an integer setting, unlike ADSB_STATISTICS
  this->set_setting("SYSTEM_STATISTICS", protocol == STATS_PROTOCOL_CSV ? "1" : "0", callback);
}

void SolderedADSBReceiver::set_adsb_rx_protocol_decoded(DecodedProtocol protocol, ResultCallback callback) {
  // A string setting on the module, confirmed with AT+ADSB_RX_PROTOCOL_DECODED=?,
  // so it takes the protocol's name rather than an index
  const char *name = decoded_protocol_to_string(protocol);
  if (name == nullptr) {
    ESP_LOGE(TAG, "Invalid ADSB_RX_PROTOCOL_DECODED value %u", (unsigned) protocol);
    if (callback)
      callback(false);
    return;
  }

  this->set_setting("ADSB_RX_PROTOCOL_DECODED", name, callback);
}

void SolderedADSBReceiver::set_adsb_rx_protocol_inc(IncMode mode, ResultCallback callback) {
  if (mode > INC_IMMEDIATE_ON_POSITION_UPDATE) {
    ESP_LOGE(TAG, "Invalid ADSB_RX_PROTOCOL_INC value %u", (unsigned) mode);
    if (callback)
      callback(false);
    return;
  }

  char value[4];
  snprintf(value, sizeof(value), "%u", (unsigned) mode);

  this->set_setting("ADSB_RX_PROTOCOL_INC", value, callback);
}

void SolderedADSBReceiver::set_adsb_statistics_protocol(StatsProtocol protocol, ResultCallback callback) {
  if (protocol > STATS_PROTOCOL_CSV) {
    ESP_LOGE(TAG, "Invalid ADSB_STATISTICS value %u", (unsigned) protocol);
    if (callback)
      callback(false);
    return;
  }

  // A string setting like ADSB_RX_PROTOCOL_DECODED, so it takes the name
  this->set_setting("ADSB_STATISTICS", protocol == STATS_PROTOCOL_CSV ? "CSV" : "None", callback);
}

// *****************************************************************************
// Section: Building the queue

SolderedADSBReceiver::Step SolderedADSBReceiver::make_command_(const std::string &body, ReplyType reply,
                                                               uint32_t timeout_ms, StepHandler handler,
                                                               bool critical) {
  Step step = make_step_(StepType::COMMAND, timeout_ms);
  step.reply = reply;
  step.body = body;
  step.handler = std::move(handler);
  step.critical = critical;
  return step;
}

SolderedADSBReceiver::Step SolderedADSBReceiver::make_step_(StepType type, uint32_t value) {
  Step step{};
  step.type = type;
  step.reply = ReplyType::TERMINAL;
  step.value = value;
  step.critical = false;
  return step;
}

void SolderedADSBReceiver::queue_expect_ok_(const std::string &body, uint32_t timeout_ms, ResultCallback callback) {
  auto handler = [body, callback](bool received, const std::string &reply) {
    bool success = received && is_ok(reply);
    if (!success)
      ESP_LOGW(TAG, "AT+%s failed: %s", body.c_str(), received ? reply.c_str() : "no answer");
    if (callback)
      callback(success);
    return success;
  };

  this->queue_.push_back(make_command_(body, ReplyType::TERMINAL, timeout_ms, handler));
}

void SolderedADSBReceiver::queue_query_(const std::string &name, ValueCallback callback) {
  auto handler = [name, callback](bool received, const std::string &reply) {
    std::string value;
    bool success = received && extract_assignment(reply, name, value);
    if (!success)
      ESP_LOGW(TAG, "AT+%s? failed: %s", name.c_str(), received ? reply.c_str() : "no answer");
    if (callback)
      callback(success, value);
    return success;
  };

  this->queue_.push_back(make_command_(name + "?", ReplyType::QUERY, AT_TIMEOUT_MS, handler));
}

void SolderedADSBReceiver::queue_multiline_(const std::string &body, uint32_t timeout_ms, ValueCallback callback) {
  auto handler = [body, callback](bool received, const std::string &reply) {
    if (!received)
      ESP_LOGW(TAG, "AT+%s got no answer", body.c_str());
    if (callback)
      callback(received, reply);
    return received;
  };

  this->queue_.push_back(make_command_(body, ReplyType::MULTILINE, timeout_ms, handler));
}

// *****************************************************************************
// Section: Running the queue

void SolderedADSBReceiver::process_queue_() {
  if (this->queue_.empty())
    return;

  Step &step = this->queue_.front();
  uint32_t now = millis();

  if (!this->step_active_) {
    this->step_active_ = true;
    this->step_start_ms_ = now;
    this->start_step_(step);
    return;
  }

  uint32_t elapsed = now - this->step_start_ms_;

  switch (step.type) {
    case StepType::COMMAND:
      if (this->reply_done_) {
        this->finish_step_(true);
      } else if (step.reply == ReplyType::MULTILINE && this->reply_any_ &&
                 now - this->last_reply_line_ms_ >= MULTILINE_QUIET_MS) {
        // Once a multi-line reply has started, a quiet spell is what ends it
        this->finish_step_(true);
      } else if (elapsed >= step.value) {
        this->finish_step_(step.reply == ReplyType::MULTILINE && this->reply_any_);
      }
      break;

    case StepType::WAIT:
      if (elapsed >= step.value)
        this->finish_step_(true);
      break;

    default:
      break;
  }
}

void SolderedADSBReceiver::start_step_(Step &step) {
  switch (step.type) {
    case StepType::COMMAND: {
      if (step.body.size() > MAX_COMMAND_LEN) {
        ESP_LOGE(TAG, "AT command body is longer than %u characters", (unsigned) MAX_COMMAND_LEN);
        this->finish_step_(false);
        return;
      }

      // Everything received so far has been decoded already, only a partial line or frame is dropped here
      this->line_len_ = 0;
      this->mavlink_.reset();
      this->reply_.clear();
      this->reply_any_ = false;
      this->reply_done_ = false;

      ESP_LOGV(TAG, "Sending AT+%s", step.body.c_str());
      std::string command = "AT+" + step.body + "\r\n";
      this->write_str(command.c_str());
      break;
    }

    case StepType::SET_BAUD:
      this->set_uart_baud_(step.value);
      this->finish_step_(true);
      break;

    case StepType::RESET_ASSERT:
      // Level first, so the pin never drives the wrong level when it turns into an output
      this->reset_pin_->digital_write(false);
      this->reset_pin_->pin_mode(gpio::FLAG_OUTPUT);
      this->finish_step_(true);
      break;

    case StepType::RESET_RELEASE:
      // Released back to high impedance rather than held at a guessed idle level
      this->reset_pin_->pin_mode(gpio::FLAG_INPUT);
      this->discard_input_();
      this->finish_step_(true);
      break;

    case StepType::WAIT:
    default:
      break;
  }
}

void SolderedADSBReceiver::finish_step_(bool received) {
  // Taken off the queue before its handler runs, so the handler can queue follow-up steps at the front
  Step step = std::move(this->queue_.front());
  this->queue_.pop_front();
  this->step_active_ = false;

  std::string reply = std::move(this->reply_);
  this->reply_.clear();
  this->reply_any_ = false;
  this->reply_done_ = false;

  bool success = step.handler ? step.handler(received, reply) : true;
  if (!success && step.critical && !this->queue_.empty()) {
    ESP_LOGW(TAG, "Dropping %u queued steps after a failed AT+%s", (unsigned) this->queue_.size(), step.body.c_str());
    this->queue_.clear();
  }
}

void SolderedADSBReceiver::set_uart_baud_(uint32_t baud_rate) {
  if (this->parent_->get_baud_rate() == baud_rate)
    return;

  ESP_LOGD(TAG, "Switching the UART to %" PRIu32 " baud", baud_rate);
  this->flush();
  this->parent_->set_baud_rate(baud_rate);
  this->parent_->load_settings(false);

  // Anything still waiting was sent at the old baud rate
  this->discard_input_();
}

void SolderedADSBReceiver::discard_input_() {
  uint8_t buffer[READ_CHUNK_SIZE];
  size_t available;
  while ((available = this->available()) > 0) {
    if (!this->read_array(buffer, std::min(available, sizeof(buffer))))
      break;
  }

  this->line_len_ = 0;
  this->mavlink_.reset();
}

// *****************************************************************************
// Section: Reading the stream

void SolderedADSBReceiver::read_input_() {
  uint8_t buffer[READ_CHUNK_SIZE];
  size_t available;

  while ((available = this->available()) > 0) {
    size_t length = std::min(available, sizeof(buffer));
    if (!this->read_array(buffer, length))
      break;

    for (size_t i = 0; i < length; i++)
      this->feed_byte_(buffer[i]);

    // Stop at the end of a reply, so a baud rate switch that follows it does not decode bytes sent at the new rate
    if (this->reply_done_)
      break;
  }
}

void SolderedADSBReceiver::feed_byte_(uint8_t byte) {
  if (this->mavlink_.busy()) {
    if (this->mavlink_.feed(byte))
      this->dispatch_mavlink_frame_();
    return;
  }

  // A MAVLink start byte can only begin a frame between lines, never inside one
  if (this->line_len_ == 0 && this->mavlink_.start(byte))
    return;

  if (this->feed_line_((char) byte)) {
    this->handle_line_(this->line_buf_);
    this->line_len_ = 0;
  }
}

bool SolderedADSBReceiver::feed_line_(char c) {
  if (c == '\r')
    return false;

  if (c == '\n') {
    if (this->line_len_ == 0)
      return false;
    this->line_buf_[this->line_len_] = '\0';
    return true;
  }

  // A line this long is garbage, start over rather than keep growing it
  if (this->line_len_ >= MAX_LINE_LEN) {
    this->line_len_ = 0;
    return false;
  }

  this->line_buf_[this->line_len_++] = c;

  return false;
}

void SolderedADSBReceiver::handle_line_(char *raw_line) {
  char *line = trim_in_place(raw_line);
  if (*line == '\0')
    return;

  if (this->line_callback_.size() > 0)
    this->line_callback_.call(std::string(line));

  // AT replies go to the command waiting for them
  if (starts_with(line, "AT+") && this->step_active_ && this->queue_.front().type == StepType::COMMAND &&
      !this->reply_done_) {
    this->collect_reply_line_(line);
    return;
  }

  // "#AS:" is checked before "#A:", which it also starts with
  if (starts_with(line, "#AS:")) {
    ADSBStats stats;
    if (parse_adsb_stats_line(line, stats))
      this->adsb_stats_callback_.call(stats);
    return;
  }

  if (starts_with(line, "#A:")) {
    Aircraft aircraft;
    if (parse_aircraft_line(line, aircraft))
      this->aircraft_callback_.call(aircraft);
    return;
  }

  if (starts_with(line, "#S:")) {
    SystemStats stats;
    if (parse_system_stats_line(line, stats))
      this->system_stats_callback_.call(stats);
    return;
  }

  ESP_LOGV(TAG, "Unknown line: %s", line);
  if (this->unknown_line_callback_.size() > 0)
    this->unknown_line_callback_.call(std::string(line));
}

void SolderedADSBReceiver::collect_reply_line_(const char *line) {
  const Step &step = this->queue_.front();

  if (this->reply_.size() < MAX_REPLY_LEN) {
    if (!this->reply_.empty())
      this->reply_ += '\n';
    this->reply_ += line;
  }
  this->reply_any_ = true;
  this->last_reply_line_ms_ = millis();

  switch (step.reply) {
    case ReplyType::TERMINAL:
      this->reply_done_ = is_terminal_line(line);
      break;
    case ReplyType::QUERY:
      this->reply_done_ = true;
      break;
    case ReplyType::PREFIX:
      // A generic AT+OK or an exit notice may come before the line waited for
      this->reply_done_ = starts_with(line, step.prefix.c_str()) || starts_with(line, "AT+ERROR");
      break;
    case ReplyType::MULTILINE:
    default:
      break;
  }
}

void SolderedADSBReceiver::dispatch_mavlink_frame_() {
  const MAVLinkFrame &frame = this->mavlink_.frame();

  this->mavlink_frame_callback_.call(frame);

  if (frame.crc_valid && frame.msgid == MAVLINK_MSG_ID_ADSB_VEHICLE && this->adsb_vehicle_callback_.size() > 0) {
    ADSBVehicle vehicle;
    if (parse_mavlink_adsb_vehicle(frame, vehicle))
      this->adsb_vehicle_callback_.call(vehicle);
  }
}

}  // namespace esphome::soldered_adsb_receiver
