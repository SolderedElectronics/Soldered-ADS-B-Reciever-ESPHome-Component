# Soldered ADS-B 1090MHz Receiver ESPHome Component

| ![ADS-B 1090MHz Receiver ESP32-S3](https://cms.soldered.com/products/333388/media/333388_photo_b98aaf.webp) |
| :---------------------------------------------------------------------------------------------------------: |
|                         [ADS-B 1090MHz Receiver ESP32-S3](https://solde.red/333388)                          |

The Soldered ADS-B 1090MHz Receiver ESP32-S3 is built around the Aerobits TT-SC1b OEM ADS-B receiver module. The module
listens for the ADS-B squitters aircraft broadcast on 1090 MHz and reports each aircraft's position, altitude, speed,
callsign and squawk over UART. It is configured over the same UART with AT commands, and streams live traffic as either
CSV or MAVLink.

External ESPHome component for the Soldered ADS-B 1090MHz Receiver. It is a port of the
[Soldered ADS-B Receiver ESP-IDF component](https://github.com/SolderedElectronics/Soldered-ADSB-Reciever-ESP-IDF-Component),
rebuilt on top of ESPHome's [`uart:`](https://esphome.io/components/uart/) bus so it never blocks the main loop.

This component covers what the base SC1 module supports: CSV and MAVLink output. Mode-A/C/S decode, USB,
multilateration timestamps and the JSON, GDL90, ASTERIX and RAW protocols are exclusive to the SC1-EXT variant and are
not included.

- **Decoding**: aircraft state vectors (`#A:`), ADS-B receiver statistics (`#AS:`) and system statistics (`#S:`) with
  CRC16 verification of every line, and MAVLink v1/v2 frames with checksum verification, decoded into ADSB_VEHICLE.
  CSV and MAVLink are told apart byte by byte, so the same configuration works whichever protocol the module is set to.
- **Settings at boot**: output protocol, report mode, statistics protocols, system log and baud rate, written only when
  given in the YAML.
- **State machine and AT commands**: RUN / CONFIGURATION / BOOTLOADER, lock, reboot, hardware reset over the RESET pin,
  every setting, help text, serial number, firmware version and device type, all callable from lambdas.

## Repository Contents

- **components/** - the ESPHome external component (Python config + C++ implementation)
  - `soldered_adsb_receiver.h/.cpp` - the component: UART stream, AT command queue, state machine
  - `adsb_receiver_csv.h/.cpp` - the CSV decoders and CRC16
  - `adsb_receiver_mavlink.h/.cpp` - the MAVLink frame parser and ADSB_VEHICLE decoder
  - `adsb_receiver_defs.h` - setting values and decoded data types
- **examples/** - example YAML configs showing how to use the component
  - `read_aircraft_csv.yaml` - default RUN state, decoded aircraft from the CSV stream
  - `device_info.yaml` - CONFIGURATION state, serial number, firmware version, `AT+HELP`, `AT+SETTINGS?`
  - `configure_protocols_and_settings.yaml` - writing every supported setting
  - `statistics.yaml` - system (`#S:`) and ADS-B (`#AS:`) statistics messages
  - `state_machine_and_lock.yaml` - full RESET -> RUN -> CONFIGURATION -> BOOTLOADER (locked) -> RUN cycle
  - `mavlink_adsb_vehicle.yaml` - MAVLink output, decoded ADSB_VEHICLE messages, generic frame automation
  - `custom_board_wiring.yaml` - driving a module wired to different pins

## Usage

Reference this repo directly from your own ESPHome YAML (no need to clone it locally). On the Soldered board the module
is wired to IO17 (module TX) and IO18 (module RX) of the ESP32-S3, with its RESET pin on IO21, and leaves the factory
sending CSV at 115200 baud:

```yaml
external_components:
  - source: github://SolderedElectronics/Soldered-ADS-B-Reciever-ESPHome-Component
    components: [soldered_adsb_receiver]

uart:
  id: adsb_uart
  rx_pin: GPIO17
  tx_pin: GPIO18
  baud_rate: 115200
  rx_buffer_size: 4096

soldered_adsb_receiver:
  id: adsb
  uart_id: adsb_uart
  reset_pin: GPIO21
  on_aircraft:
    - lambda: |-
        if (x.has_position)
          ESP_LOGI("aircraft", "%s at %.5f, %.5f", x.icao, x.lat, x.lon);
```

See [`examples/read_aircraft_csv.yaml`](examples/read_aircraft_csv.yaml) for a full working example.

**Reading data:** the module keeps sending whether or not anybody is listening, and the component drains the UART every
loop. Give the `uart:` bus a `rx_buffer_size` of a few kB, so dense traffic never overflows it between two loop
iterations; at 921600 or 3000000 baud make it bigger still.

### Configuration variables

- **uart_id** (*Optional*, [ID](https://esphome.io/guides/configuration-types#config-id)): the `uart:` bus the module
  is on. Its `baud_rate` is the RUN baud rate the module is expected at.
- **reset_pin** (*Optional*, [Pin Schema](https://esphome.io/guides/configuration-types#config-pin-schema)): pin wired
  to the RESET pin of the module. It is left as a high impedance input and only driven low for the length of a
  `hardware_reset()` pulse.

#### Settings

Every setting given here is written to the module at boot: the component moves it into the CONFIGURATION state, writes
the settings, and returns it to RUN, all in the background. Settings left out are not touched. The module keeps its
settings across power cycles. Writing settings needs the `tx_pin` of the `uart:` bus.

- **decoded_protocol** (*Optional*, string): the protocol aircraft are reported in, `ADSB_RX_PROTOCOL_DECODED`. One of
  `CSV`, `MAVLINK`, `NONE`.
- **report_mode** (*Optional*, string): how often aircraft are reported, `ADSB_RX_PROTOCOL_INC`. One of
  `ALWAYS_ONCE_PER_SECOND`, `ONCE_PER_SECOND_IF_UPDATED`, `IMMEDIATE_ON_POSITION_UPDATE`.
- **adsb_statistics** (*Optional*, string): the protocol of the `#AS:` ADS-B statistics, `ADSB_STATISTICS`. One of
  `CSV`, `NONE`.
- **system_statistics** (*Optional*, string): the protocol of the `#S:` system statistics, `SYSTEM_STATISTICS`. One of
  `CSV`, `NONE`.
- **system_log** (*Optional*, boolean): whether the module sends system log lines, `SYSTEM_LOG`.
- **baud_rate** (*Optional*, int): the RUN baud rate, `BAUDRATE`. One of `57600`, `115200`, `921600`, `3000000`. Must
  match the `baud_rate` of the `uart:` bus.

If the module does not answer at the `uart:` baud rate, the component tries every other baud rate it supports, so
changing `baud_rate` together with the `uart:` one is enough to move a module over.

#### Automations

Each automation gets the decoded message as `x`. The structs are described in
[`adsb_receiver_defs.h`](components/soldered_adsb_receiver/adsb_receiver_defs.h).

- **on_aircraft** - a decoded `#A:` aircraft message, `x` is an `Aircraft`: `icao`, `callsign`, `squawk`, `lat`/`lon`,
  `alt_baro`, `alt_geo`, `track`, `vel_h`, `vel_v`, signal strength and more. A field the module had no data for has its
  `has_` flag false. `x.on_ground()`, `x.is_military()` and `x.has_flag()` read the flags, `x.crc_valid` tells whether
  the line passed its CRC check.
- **on_adsb_stats** - a decoded `#AS:` message, `x` is an `ADSBStats`: `fpss`, `fpsac`, `calib`.
- **on_system_stats** - a decoded `#S:` message, `x` is a `SystemStats`: `cpu_load`, `uptime`.
- **on_adsb_vehicle** - a decoded MAVLink ADSB_VEHICLE message that passed its checksum, `x` is an `ADSBVehicle`.
  `x.lat_degrees()`, `x.lon_degrees()`, `x.altitude_meters()` and `x.has_flag()` help reading it.
- **on_mavlink_frame** - every MAVLink frame, whatever its message id, `x` is a `MAVLinkFrame`.
- **on_line** - every line received, AT replies included, `x` is a `std::string`. The quickest way to see what is on
  the wire.
- **on_unknown_line** - every line that is not a known message or a reply to a command.

### Commands from lambdas

Settings beyond the ones above, and the state machine, are reachable from lambdas through `id(adsb)`. Every command is
queued and runs in order in the background, so each returns right away and reports back through an optional callback,
`[](bool success) {...}` or `[](bool success, const std::string &value) {...}` for the ones that read something:

```yaml
esphome:
  on_boot:
    then:
      - lambda: |-
          id(adsb).enter_configuration();
          id(adsb).get_firmware_version([](bool success, const std::string &value) {
            ESP_LOGI("adsb", "Firmware: %s", value.c_str());
          });
          id(adsb).exit_configuration();
```

- **State machine**: `enter_configuration()`, `exit_configuration()`, `get_config_state()`, `reboot()`,
  `reboot_to_bootloader()`, `set_lock()`, `get_lock()`, `is_bootloader()`, `hardware_reset(pulse_ms, resync_to_115200)`
  and `queue_delay(ms)` to wait between steps.
- **Generic settings**: `set_setting(name, value)`, `get_setting(name)`, `get_setting_description(name)`,
  `get_all_settings()`, `get_full_help()`, `reset_settings_to_default()`, or any AT command with
  `send_at(body, callback, timeout_ms, wait_for_prefix)`.
- **Device info**: `get_serial_number()`, `get_firmware_version()`, `get_device_type()`, `test()`.
- **Typed settings**: `set_adsb_rx_protocol_decoded()`, `set_adsb_rx_protocol_inc()`, `set_adsb_statistics_protocol()`,
  `set_system_statistics_protocol()`, `set_system_log()`, `set_baudrate()`.

Settings can only be read and written in the CONFIGURATION state, which always runs at 115200 baud;
`enter_configuration()` gets there and switches the local UART to match, and `exit_configuration()` goes back to RUN at
the `uart:` baud rate, or the one `set_baudrate()` last wrote. If `enter_configuration()` fails, everything queued after
it is dropped. See [`examples/device_info.yaml`](examples/device_info.yaml) and
[`examples/state_machine_and_lock.yaml`](examples/state_machine_and_lock.yaml).

### Hardware design

You can find hardware design for this board in _Soldered ADS-B 1090MHz Receiver ESP32-S3_ hardware repository.

### Documentation

Access library documentation [here](https://docs.soldered.com/).

### About Soldered

<img src="https://raw.githubusercontent.com/SolderedElectronics/Soldered-Generic-Arduino-Library/dev/extras/Soldered-logo-color.png" alt="soldered-logo" width="500"/>

At Soldered, we design and manufacture a wide selection of electronic products to help you turn your ideas into acts and bring you one step closer to your final project. Our products are intented for makers and crafted in-house by our experienced team in Osijek, Croatia. We believe that sharing is a crucial element for improvement and innovation, and we work hard to stay connected with all our makers regardless of their skill or experience level. Therefore, all our products are open-source. Finally, we always have your back. If you face any problem concerning either your shopping experience or your electronics project, our team will help you deal with it, offering efficient customer service and cost-free technical support anytime. Some of those might be useful for you:

- [Web Store](https://www.soldered.com/shop)
- [Tutorials & Projects](https://soldered.com/learn)
- [Documentation](https://docs.soldered.com)

### Open-source license

Soldered invests vast amounts of time into hardware & software for these products, which are all open-source. Please support future development by buying one of our products.

Check license details in the LICENSE file. Long story short, use these open-source files for any purpose you want to, as long as you apply the same open-source licence to it and disclose the original source. No warranty - all designs in this repository are distributed in the hope that they will be useful, but without any warranty. They are provided "AS IS", therefore without warranty of any kind, either expressed or implied. The entire quality and performance of what you do with the contents of this repository are your responsibility. In no event, Soldered (TAVU) will be liable for your damages, losses, including any general, special, incidental or consequential damage arising out of the use or inability to use the contents of this repository.

## Have fun!

And thank you from your fellow makers at Soldered Electronics.
