/**
 * @file adsb_receiver_defs.h
 * @brief Definitions, setting values and decoded data types for the Aerobits TT-SC1b
 *
 * The AT command set, the CSV data protocol and the settings described here
 * follow the Aerobits OEM SC1/SC1-EXT datasheet. The MAVLink ADSB_VEHICLE
 * message follows the MAVLink common message set:
 * https://mavlink.io/en/messages/common.html#ADSB_VEHICLE
 *
 * Only what the base SC1 module supports is covered: CSV and MAVLink output.
 * Mode-A/C/S decode, multilateration timestamps and the JSON, GDL90, ASTERIX
 * and RAW protocols are exclusive to the SC1-EXT variant.
 *
 * @author Soldered Electronics
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::soldered_adsb_receiver {

// *****************************************************************************
// Section: Limits

/// Fixed baud rate of the CONFIGURATION and BOOTLOADER states
static constexpr uint32_t CONFIG_BAUD_RATE = 115200;

/// Longest line kept while assembling CSV and AT lines, longer ones are dropped
static constexpr size_t MAX_LINE_LEN = 512;

/// Longest AT command body accepted by SolderedADSBReceiver::send_at()
static constexpr size_t MAX_COMMAND_LEN = 128;

/// ICAO address, 6 hex digits, string terminator included
static constexpr size_t ICAO_SIZE = 8;
/// Callsign, up to 8 characters, string terminator included
static constexpr size_t CALLSIGN_SIZE = 9;
/// Squawk, 4 octal digits, string terminator included
static constexpr size_t SQUAWK_SIZE = 8;

/// Largest payload a MAVLink frame can carry
static constexpr size_t MAVLINK_MAX_PAYLOAD_LEN = 255;
/// MAVLink message id of HEARTBEAT
static constexpr uint32_t MAVLINK_MSG_ID_HEARTBEAT = 0;
/// MAVLink message id of ADSB_VEHICLE
static constexpr uint32_t MAVLINK_MSG_ID_ADSB_VEHICLE = 246;

// *****************************************************************************
// Section: Settings

/// Values of the ADSB_RX_PROTOCOL_DECODED setting supported by the SC1
enum DecodedProtocol : uint8_t {
  DECODED_PROTOCOL_NONE = 0,     ///< No decoded aircraft output
  DECODED_PROTOCOL_CSV = 1,      ///< "#A:" CSV lines, the default
  DECODED_PROTOCOL_MAVLINK = 2,  ///< MAVLink ADSB_VEHICLE messages
};

/// Values of the SYSTEM_STATISTICS and ADSB_STATISTICS settings supported by the SC1
enum StatsProtocol : uint8_t {
  STATS_PROTOCOL_NONE = 0,  ///< No statistics output, the default
  STATS_PROTOCOL_CSV = 1,   ///< "#S:" or "#AS:" CSV lines once a second
};

/// Values of the BAUDRATE setting, the baud rate of the main UART in the RUN state
enum Baudrate : uint8_t {
  BAUDRATE_115200 = 0,   ///< 115200 baud, the default
  BAUDRATE_921600 = 1,   ///< 921600 baud
  BAUDRATE_3000000 = 2,  ///< 3000000 baud
  BAUDRATE_57600 = 3,    ///< 57600 baud
};

/// Values of the ADSB_RX_PROTOCOL_INC setting, how often aircraft reports are sent
enum IncMode : uint8_t {
  INC_ALWAYS_ONCE_PER_SECOND = 0,        ///< Every aircraft once a second
  INC_ONCE_PER_SECOND_IF_UPDATED = 1,    ///< Once a second, only aircraft that changed
  INC_IMMEDIATE_ON_POSITION_UPDATE = 2,  ///< As soon as a new position comes in
};

/// Values of AT+CONFIG?, the state the module reports being in
enum ConfigState : uint8_t {
  CONFIG_STATE_RUN = 0,              ///< RUN state
  CONFIG_STATE_CONFIG_115200 = 1,    ///< CONFIGURATION state at 115200 baud
  CONFIG_STATE_CONFIG_SET_BAUD = 2,  ///< CONFIGURATION state at the BAUDRATE setting
};

/**
 * @brief Baud rate in bits per second of a BAUDRATE setting value
 *
 * @param[in] baudrate Setting value
 *
 * @return Baud rate in bits per second
 */
inline uint32_t baudrate_to_bps(Baudrate baudrate) {
  switch (baudrate) {
    case BAUDRATE_921600:
      return 921600;
    case BAUDRATE_3000000:
      return 3000000;
    case BAUDRATE_57600:
      return 57600;
    case BAUDRATE_115200:
    default:
      return 115200;
  }
}

// *****************************************************************************
// Section: CSV messages

/// FLAGS bits of a "#A:" aircraft message
static constexpr uint16_t AIRCRAFT_FLAG_ON_GROUND = 0x0001;        ///< Aircraft is on the ground
static constexpr uint16_t AIRCRAFT_FLAG_MILITARY = 0x0002;         ///< Aircraft is military
static constexpr uint16_t AIRCRAFT_FLAG_UPDATE_ALT_BARO = 0x0100;  ///< Barometric altitude was updated
static constexpr uint16_t AIRCRAFT_FLAG_UPDATE_POSITION = 0x0200;  ///< Position was updated
static constexpr uint16_t AIRCRAFT_FLAG_UPDATE_TRACK = 0x0400;     ///< Track was updated
static constexpr uint16_t AIRCRAFT_FLAG_UPDATE_VELO_H = 0x0800;    ///< Horizontal velocity was updated
static constexpr uint16_t AIRCRAFT_FLAG_UPDATE_VELO_V = 0x1000;    ///< Vertical velocity was updated
static constexpr uint16_t AIRCRAFT_FLAG_UPDATE_ALT_GEO = 0x2000;   ///< Geometric altitude was updated

/**
 * @brief Decoded "#A:" aircraft state vector
 *
 * Any field the module had no data for is sent empty; its `has_` flag is then
 * false and the value is left at zero.
 */
struct Aircraft {
  char icao[ICAO_SIZE];          ///< ICAO 24-bit address, hex string
  uint16_t flags;                ///< AIRCRAFT_FLAG_ bits
  char callsign[CALLSIGN_SIZE];  ///< Callsign, empty if not known
  char squawk[SQUAWK_SIZE];      ///< Squawk, empty if not known
  bool has_position;             ///< lat and lon are valid
  double lat;                    ///< Latitude in degrees
  double lon;                    ///< Longitude in degrees
  bool has_alt_baro;             ///< alt_baro is valid
  int32_t alt_baro;              ///< Barometric altitude in feet
  bool has_track;                ///< track is valid
  float track;                   ///< Track in degrees, 0 to 360
  bool has_vel_h;                ///< vel_h is valid
  float vel_h;                   ///< Horizontal velocity in knots
  bool has_vel_v;                ///< vel_v is valid
  int32_t vel_v;                 ///< Vertical velocity in ft/min
  bool has_sig_s;                ///< sig_s is valid
  int16_t sig_s;                 ///< Signal strength in dBm
  bool has_sig_q;                ///< sig_q is valid
  uint8_t sig_q;                 ///< Signal quality in dB
  uint16_t fps;                  ///< Raw Mode-S frames received last second
  bool has_nic_nac;              ///< nic_nac is valid
  uint16_t nic_nac;              ///< Raw NIC/NAC bitfield
  bool has_alt_geo;              ///< alt_geo is valid
  int32_t alt_geo;               ///< Geometric altitude in feet
  bool has_ecat;                 ///< ecat is valid
  uint8_t ecat;                  ///< Emitter category code
  uint16_t crc;                  ///< CRC sent with the line
  bool crc_valid;                ///< CRC sent matches the one worked out

  /// Check a bit of the flags, one of the AIRCRAFT_FLAG_ bits
  bool has_flag(uint16_t flag) const { return (this->flags & flag) != 0; }
  /// Check whether the aircraft is on the ground
  bool on_ground() const { return this->has_flag(AIRCRAFT_FLAG_ON_GROUND); }
  /// Check whether the aircraft is military
  bool is_military() const { return this->has_flag(AIRCRAFT_FLAG_MILITARY); }
};

/// Decoded "#AS:" ADS-B receiver statistics
struct ADSBStats {
  uint32_t fpss;   ///< Mode-S frames received per second
  uint32_t fpsac;  ///< Mode-A/C frames received per second
  float calib;     ///< Real microcontroller frequency, calibrated against the GNSS PPS
  uint16_t crc;    ///< CRC sent with the line
  bool crc_valid;  ///< CRC sent matches the one worked out
};

/// Decoded "#S:" system statistics
struct SystemStats {
  float cpu_load;   ///< CPU load in percent
  uint32_t uptime;  ///< Uptime in seconds
  uint16_t crc;     ///< CRC sent with the line
  bool crc_valid;   ///< CRC sent matches the one worked out
};

// *****************************************************************************
// Section: MAVLink messages

/**
 * @brief One raw MAVLink v1 or v2 frame, whatever its message id
 *
 * `crc_valid` can only be true for message ids whose CRC_EXTRA byte is known,
 * HEARTBEAT and ADSB_VEHICLE. Any other message stays false even when the frame
 * is fine, since its checksum cannot be checked without that constant.
 */
struct MAVLinkFrame {
  uint8_t version;                           ///< 1 or 2
  uint8_t sysid;                             ///< System id of the sender
  uint8_t compid;                            ///< Component id of the sender
  uint8_t seq;                               ///< Sequence number
  uint32_t msgid;                            ///< Message id
  uint8_t payload[MAVLINK_MAX_PAYLOAD_LEN];  ///< Payload as received
  uint8_t payload_len;                       ///< Bytes in payload
  bool crc_valid;                            ///< Checksum was checked and matched
};

/// Values of the ADSB_VEHICLE altitude_type field
enum AltitudeType : uint8_t {
  ALTITUDE_TYPE_PRESSURE_QNH = 0,  ///< Barometric altitude
  ALTITUDE_TYPE_GEOMETRIC = 1,     ///< GNSS altitude
};

/// Values of the ADSB_VEHICLE emitter_type field
enum EmitterType : uint8_t {
  EMITTER_NO_INFO = 0,
  EMITTER_LIGHT = 1,
  EMITTER_SMALL = 2,
  EMITTER_LARGE = 3,
  EMITTER_HIGH_VORTEX_LARGE = 4,
  EMITTER_HEAVY = 5,
  EMITTER_HIGHLY_MANUV = 6,
  EMITTER_ROTORCRAFT = 7,
  EMITTER_UNASSIGNED = 8,
  EMITTER_GLIDER = 9,
  EMITTER_LIGHTER_AIR = 10,
  EMITTER_PARACHUTE = 11,
  EMITTER_ULTRA_LIGHT = 12,
  EMITTER_UNASSIGNED2 = 13,
  EMITTER_UAV = 14,
  EMITTER_SPACE = 15,
  EMITTER_UNASSIGNED3 = 16,
  EMITTER_EMERGENCY_SURFACE = 17,
  EMITTER_SERVICE_SURFACE = 18,
  EMITTER_POINT_OBSTACLE = 19,
};

/// Bits of the ADSB_VEHICLE flags field
static constexpr uint16_t VEHICLE_FLAG_VALID_COORDS = 0x0001;             ///< lat and lon are valid
static constexpr uint16_t VEHICLE_FLAG_VALID_ALTITUDE = 0x0002;           ///< altitude is valid
static constexpr uint16_t VEHICLE_FLAG_VALID_HEADING = 0x0004;            ///< heading is valid
static constexpr uint16_t VEHICLE_FLAG_VALID_VELOCITY = 0x0008;           ///< hor_velocity is valid
static constexpr uint16_t VEHICLE_FLAG_VALID_CALLSIGN = 0x0010;           ///< callsign is valid
static constexpr uint16_t VEHICLE_FLAG_VALID_SQUAWK = 0x0020;             ///< squawk is valid
static constexpr uint16_t VEHICLE_FLAG_SIMULATED = 0x0040;                ///< Simulated aircraft
static constexpr uint16_t VEHICLE_FLAG_VERTICAL_VELOCITY_VALID = 0x0080;  ///< ver_velocity is valid
static constexpr uint16_t VEHICLE_FLAG_BARO_VALID = 0x0100;               ///< Barometric altitude is valid
static constexpr uint16_t VEHICLE_FLAG_SOURCE_UAT = 0x8000;               ///< Received over UAT, not 1090ES

/// Decoded MAVLink ADSB_VEHICLE message, one aircraft's state
struct ADSBVehicle {
  uint32_t icao_address;         ///< ICAO 24-bit address
  int32_t lat;                   ///< Latitude in degrees * 1e7
  int32_t lon;                   ///< Longitude in degrees * 1e7
  AltitudeType altitude_type;    ///< What altitude is measured against
  int32_t altitude;              ///< Altitude in mm
  uint16_t heading;              ///< Heading in centidegrees
  uint16_t hor_velocity;         ///< Horizontal velocity in cm/s
  int16_t ver_velocity;          ///< Vertical velocity in cm/s, positive up
  char callsign[CALLSIGN_SIZE];  ///< Callsign, NUL terminated
  EmitterType emitter_type;      ///< Kind of aircraft
  uint8_t tslc;                  ///< Seconds since last contact
  uint16_t flags;                ///< VEHICLE_FLAG_ bits
  uint16_t squawk;               ///< Squawk code

  /// Check a bit of the flags, one of the VEHICLE_FLAG_ bits
  bool has_flag(uint16_t flag) const { return (this->flags & flag) != 0; }
  /// Latitude in degrees
  double lat_degrees() const { return this->lat / 1e7; }
  /// Longitude in degrees
  double lon_degrees() const { return this->lon / 1e7; }
  /// Altitude in metres
  double altitude_meters() const { return this->altitude / 1000.0; }
};

}  // namespace esphome::soldered_adsb_receiver
