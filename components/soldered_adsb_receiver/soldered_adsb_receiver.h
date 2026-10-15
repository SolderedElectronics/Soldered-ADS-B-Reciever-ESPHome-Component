/**
 * @file soldered_adsb_receiver.h
 * @brief Public API for the soldered_adsb_receiver ESPHome component
 *
 * ESPHome driver for the Soldered ADS-B 1090MHz Receiver, built around the
 * Aerobits TT-SC1b OEM ADS-B receiver module. The module listens for the
 * squitters aircraft broadcast on 1090 MHz and reports every aircraft's
 * position, altitude, speed, callsign and squawk over UART, either as CSV
 * lines or as MAVLink frames. It is configured over the same UART with AT
 * commands.
 *
 * The component reads the module through an ESPHome `uart:` bus. It decodes
 * the CSV and the MAVLink stream, telling them apart byte by byte, and hands
 * every decoded message to the `on_*` automations. AT commands are queued and
 * run one after another from loop() without ever blocking it, so every command
 * method below returns right away and reports its outcome through an optional
 * callback.
 *
 * @author Soldered Electronics
 */

#pragma once

#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"
#include "esphome/core/optional.h"

#include "adsb_receiver_csv.h"
#include "adsb_receiver_defs.h"
#include "adsb_receiver_mavlink.h"

namespace esphome::soldered_adsb_receiver {

/// Called once a queued command is done; `success` is false on a timeout, AT+ERROR or an unexpected reply
using ResultCallback = std::function<void(bool success)>;

/// Called once a queued query is done with the value or reply text it returned
using ValueCallback = std::function<void(bool success, const std::string &value)>;

/**
 * @brief Soldered ADS-B 1090MHz Receiver, Aerobits TT-SC1b module on a UART bus
 */
class SolderedADSBReceiver : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  // ***************************************************************************
  // Section: Configuration from YAML

  /// Pin wired to the RESET pin of the module, driven low only for the length of a reset pulse
  void set_reset_pin(GPIOPin *reset_pin) { this->reset_pin_ = reset_pin; }
  /// ADSB_RX_PROTOCOL_DECODED written at boot
  void set_boot_decoded_protocol(DecodedProtocol protocol) { this->boot_decoded_protocol_ = protocol; }
  /// ADSB_RX_PROTOCOL_INC written at boot
  void set_boot_inc_mode(IncMode mode) { this->boot_inc_mode_ = mode; }
  /// ADSB_STATISTICS written at boot
  void set_boot_adsb_statistics(StatsProtocol protocol) { this->boot_adsb_statistics_ = protocol; }
  /// SYSTEM_STATISTICS written at boot
  void set_boot_system_statistics(StatsProtocol protocol) { this->boot_system_statistics_ = protocol; }
  /// SYSTEM_LOG written at boot
  void set_boot_system_log(bool enable) { this->boot_system_log_ = enable; }
  /// BAUDRATE written at boot
  void set_boot_baudrate(Baudrate baudrate) { this->boot_baudrate_ = baudrate; }

  // ***************************************************************************
  // Section: Automation callbacks

  /// Decoded "#A:" aircraft messages
  template<typename F> void add_on_aircraft_callback(F &&callback) {
    this->aircraft_callback_.add(std::forward<F>(callback));
  }
  /// Decoded "#AS:" ADS-B statistics messages
  template<typename F> void add_on_adsb_stats_callback(F &&callback) {
    this->adsb_stats_callback_.add(std::forward<F>(callback));
  }
  /// Decoded "#S:" system statistics messages
  template<typename F> void add_on_system_stats_callback(F &&callback) {
    this->system_stats_callback_.add(std::forward<F>(callback));
  }
  /// Decoded MAVLink ADSB_VEHICLE messages, only for frames that pass checksum verification
  template<typename F> void add_on_adsb_vehicle_callback(F &&callback) {
    this->adsb_vehicle_callback_.add(std::forward<F>(callback));
  }
  /// Every MAVLink frame received, whatever its message id
  template<typename F> void add_on_mavlink_frame_callback(F &&callback) {
    this->mavlink_frame_callback_.add(std::forward<F>(callback));
  }
  /// Every line received, including AT replies, before it is decoded
  template<typename F> void add_on_line_callback(F &&callback) { this->line_callback_.add(std::forward<F>(callback)); }
  /// Lines that are not a known message and not a reply to a queued command
  template<typename F> void add_on_unknown_line_callback(F &&callback) {
    this->unknown_line_callback_.add(std::forward<F>(callback));
  }

  // ***************************************************************************
  // Section: State machine
  //
  // Every method from here on queues its work and returns right away. Queued
  // work runs in order, one step at a time, from loop().

  /**
   * @brief Reset the module by pulsing its RESET pin low
   *
   * Afterwards the module restarts: it sits in the BOOTLOADER state at a fixed
   * 115200 baud for about 3 s and then moves on to RUN on its own, unless it is
   * locked. Anything received before the reset is thrown away.
   *
   * @param[in] pulse_ms How long to hold the module in reset
   * @param[in] resync_to_115200 Also switch the local UART to 115200 baud, which
   *            is what the module comes back up at
   *
   * @return false if no reset pin is configured, nothing is queued then
   */
  bool hardware_reset(uint32_t pulse_ms = 20, bool resync_to_115200 = true);

  /**
   * @brief Move from RUN to CONFIGURATION
   *
   * Sends AT+CONFIG=1, switches the local UART to 115200 baud, the fixed baud
   * rate of the CONFIGURATION state, and confirms the change with AT+CONFIG?.
   * If the module does not answer at the RUN baud rate it is expected at, every
   * other baud rate it supports is tried in turn. Settings can only be read and
   * written in CONFIGURATION.
   *
   * If the module cannot be reached, everything queued after this is dropped,
   * since none of it would work.
   *
   * @param[in] callback Called with the outcome, or nullptr
   */
  void enter_configuration(ResultCallback callback = nullptr);

  /**
   * @brief Move from CONFIGURATION back to RUN
   *
   * Sends AT+CONFIG=0, switches the local UART to the RUN baud rate, the `uart:`
   * baud rate or whatever set_baudrate() last wrote, and confirms the change
   * with AT+CONFIG?.
   *
   * @param[in] callback Called with the outcome, or nullptr
   */
  void exit_configuration(ResultCallback callback = nullptr);

  /// Read AT+CONFIG?, the state the module is in, one of ConfigState as text
  void get_config_state(ValueCallback callback);

  /// Reboot into the BOOTLOADER state, AT+REBOOT_BOOTLOADER; also sets the lock, so the module stays there
  void reboot_to_bootloader(ResultCallback callback = nullptr);

  /// Reboot the module, AT+REBOOT
  void reboot(ResultCallback callback = nullptr);

  /// Set or release the lock that holds the module in BOOTLOADER, AT+LOCK
  void set_lock(bool locked, ResultCallback callback = nullptr);

  /// Read the lock, AT+LOCK?, "1" if locked
  void get_lock(ValueCallback callback);

  /// Check whether the module is in the BOOTLOADER state, AT+BOOT?, "1" if it is
  void is_bootloader(ValueCallback callback);

  /// Wait before running whatever is queued next, without blocking loop()
  void queue_delay(uint32_t ms);

  /// Check whether queued work is still waiting to run
  bool is_busy() const { return !this->queue_.empty(); }

  // ***************************************************************************
  // Section: AT command transport

  /**
   * @brief Send one AT command and collect the reply
   *
   * Sends `AT+<body>\r\n` and collects every `AT+` line the module sends back,
   * joined with '\n', until a terminal line arrives. Terminal lines are AT+OK,
   * AT+ERROR and the state markers AT+CONFIG_START, AT+RUN_START and
   * AT+BOOTLOADER_START. A query, a body ending in '?', only gets a single
   * `AT+NAME=value` reply, so for a query the first reply line is terminal.
   *
   * @param[in] body Command without the leading "AT+" and without a line ending,
   *            such as "SERIAL_NUMBER?", at most MAX_COMMAND_LEN characters
   * @param[in] callback Called with the reply lines, or nullptr
   * @param[in] timeout_ms How long to wait for a terminal line
   * @param[in] wait_for_prefix If not empty, keep waiting past a generic AT+OK
   *            and only stop on a line starting with this prefix, or on AT+ERROR
   */
  void send_at(const std::string &body, ValueCallback callback = nullptr, uint32_t timeout_ms = 500,
               const std::string &wait_for_prefix = "");

  /// Check whether a reply holds AT+OK
  static bool is_ok(const std::string &reply);

  /**
   * @brief Check whether a reply holds AT+ERROR
   *
   * @param[in] reply Reply collected by send_at()
   * @param[out] description Text in parentheses after AT+ERROR, or nullptr if not wanted
   *
   * @return true if the reply holds AT+ERROR
   */
  static bool is_error(const std::string &reply, std::string *description = nullptr);

  // ***************************************************************************
  // Section: Generic settings and commands
  //
  // All of these only work in the CONFIGURATION state, see enter_configuration().

  /// Write a setting, AT+<name>=<value>
  void set_setting(const std::string &name, const std::string &value, ResultCallback callback = nullptr);
  /// Read a setting, AT+<name>?
  void get_setting(const std::string &name, ValueCallback callback);
  /// Read the description of a setting, AT+<name>=?, reply lines joined with '\n'
  void get_setting_description(const std::string &name, ValueCallback callback);
  /// List every setting with its value, AT+SETTINGS?, reply lines joined with '\n'
  void get_all_settings(ValueCallback callback);
  /// Read the full help text, AT+HELP, reply lines joined with '\n'
  void get_full_help(ValueCallback callback);
  /// Put every setting back to its default, AT+SETTINGS_DEFAULT
  void reset_settings_to_default(ResultCallback callback = nullptr);
  /// Read the serial number, AT+SERIAL_NUMBER?
  void get_serial_number(ValueCallback callback);
  /// Read the firmware version, AT+FIRMWARE_VERSION?
  void get_firmware_version(ValueCallback callback);
  /// Read the device type, AT+DEVICE?
  void get_device_type(ValueCallback callback);
  /// Check the connection, AT+TEST, succeeds on AT+OK
  void test(ResultCallback callback = nullptr);

  // ***************************************************************************
  // Section: Typed settings
  //
  // The settings the SC1 supports. Like the generic ones, they only work in the
  // CONFIGURATION state.

  /// Set BAUDRATE; takes effect on the next exit_configuration(), which then switches to it
  void set_baudrate(Baudrate baudrate, ResultCallback callback = nullptr);
  /// Set SYSTEM_LOG, whether the module sends system log lines
  void set_system_log(bool enable, ResultCallback callback = nullptr);
  /// Set SYSTEM_STATISTICS, the protocol of the "#S:" system statistics
  void set_system_statistics_protocol(StatsProtocol protocol, ResultCallback callback = nullptr);
  /// Set ADSB_RX_PROTOCOL_DECODED, the protocol aircraft are reported in
  void set_adsb_rx_protocol_decoded(DecodedProtocol protocol, ResultCallback callback = nullptr);
  /// Set ADSB_RX_PROTOCOL_INC, how often aircraft are reported
  void set_adsb_rx_protocol_inc(IncMode mode, ResultCallback callback = nullptr);
  /// Set ADSB_STATISTICS, the protocol of the "#AS:" ADS-B statistics
  void set_adsb_statistics_protocol(StatsProtocol protocol, ResultCallback callback = nullptr);

 protected:
  /// What a queued step does
  enum class StepType : uint8_t {
    COMMAND,        ///< Send an AT command and collect the reply
    SET_BAUD,       ///< Switch the local UART baud rate
    WAIT,           ///< Wait
    RESET_ASSERT,   ///< Drive the RESET pin low
    RESET_RELEASE,  ///< Release the RESET pin back to high impedance
  };

  /// What ends the reply to a COMMAND step
  enum class ReplyType : uint8_t {
    TERMINAL,   ///< AT+OK, AT+ERROR or a state marker
    QUERY,      ///< The first AT line
    PREFIX,     ///< A line starting with Step::prefix, or AT+ERROR
    MULTILINE,  ///< A quiet spell after the first AT line
  };

  /**
   * @brief Called once a step is done
   *
   * `received` is false when a COMMAND step got no complete reply in time.
   * Returns false when the step failed, which drops the rest of the queue if
   * the step is critical.
   */
  using StepHandler = std::function<bool(bool received, const std::string &reply)>;

  /// One queued step
  struct Step {
    StepType type;
    ReplyType reply;
    uint32_t value;  ///< Timeout of a COMMAND, baud rate of a SET_BAUD, duration of a WAIT, in ms
    std::string body;
    std::string prefix;
    StepHandler handler;
    bool critical;
  };

  void apply_boot_settings_();

  // Building the queue
  static Step make_command_(const std::string &body, ReplyType reply, uint32_t timeout_ms, StepHandler handler,
                            bool critical = false);
  static Step make_step_(StepType type, uint32_t value);
  void queue_expect_ok_(const std::string &body, uint32_t timeout_ms, ResultCallback callback);
  void queue_query_(const std::string &name, ValueCallback callback);
  void queue_multiline_(const std::string &body, uint32_t timeout_ms, ValueCallback callback);
  void queue_enter_attempt_(std::vector<uint32_t> baud_rates, size_t index, ResultCallback callback, bool front);
  StepHandler make_state_check_(bool enter, ResultCallback callback);

  // Running the queue
  void process_queue_();
  void start_step_(Step &step);
  void finish_step_(bool received);
  void set_uart_baud_(uint32_t baud_rate);
  void discard_input_();

  // Reading the stream
  void read_input_();
  void feed_byte_(uint8_t byte);
  bool feed_line_(char c);
  void handle_line_(char *raw_line);
  void collect_reply_line_(const char *line);
  void dispatch_mavlink_frame_();

  GPIOPin *reset_pin_{nullptr};
  uint32_t run_baud_rate_{CONFIG_BAUD_RATE};

  optional<DecodedProtocol> boot_decoded_protocol_{};
  optional<IncMode> boot_inc_mode_{};
  optional<StatsProtocol> boot_adsb_statistics_{};
  optional<StatsProtocol> boot_system_statistics_{};
  optional<bool> boot_system_log_{};
  optional<Baudrate> boot_baudrate_{};

  char line_buf_[MAX_LINE_LEN + 1]{};
  size_t line_len_{0};
  MAVLinkParser mavlink_;

  std::deque<Step> queue_;
  bool step_active_{false};
  uint32_t step_start_ms_{0};
  std::string reply_;
  bool reply_any_{false};
  bool reply_done_{false};
  uint32_t last_reply_line_ms_{0};

  CallbackManager<void(const Aircraft &)> aircraft_callback_;
  CallbackManager<void(const ADSBStats &)> adsb_stats_callback_;
  CallbackManager<void(const SystemStats &)> system_stats_callback_;
  CallbackManager<void(const ADSBVehicle &)> adsb_vehicle_callback_;
  CallbackManager<void(const MAVLinkFrame &)> mavlink_frame_callback_;
  CallbackManager<void(const std::string &)> line_callback_;
  CallbackManager<void(const std::string &)> unknown_line_callback_;
};

}  // namespace esphome::soldered_adsb_receiver
