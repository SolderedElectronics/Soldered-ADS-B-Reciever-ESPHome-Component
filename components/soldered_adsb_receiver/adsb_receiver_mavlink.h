/**
 * @file adsb_receiver_mavlink.h
 * @brief MAVLink frame parser and ADSB_VEHICLE decoder for the Aerobits TT-SC1b
 *
 * The module sends MAVLink v1 or v2 frames when its decoded output is set to
 * MAVLink. The parser here is fed one byte at a time by the component.
 *
 * @author Soldered Electronics
 */

#pragma once

#include <cstdint>
#include "adsb_receiver_defs.h"

namespace esphome::soldered_adsb_receiver {

/// Start byte of a MAVLink v1 frame
static constexpr uint8_t MAVLINK_V1_STX = 0xFE;
/// Start byte of a MAVLink v2 frame
static constexpr uint8_t MAVLINK_V2_STX = 0xFD;

/**
 * @brief MAVLink v1 and v2 frame parser
 */
class MAVLinkParser {
 public:
  /// Put the parser back into its idle state
  void reset() { this->state_ = STATE_IDLE; }

  /// Check whether the parser is inside a frame, so the next byte belongs to it
  bool busy() const { return this->state_ != STATE_IDLE; }

  /**
   * @brief Start a frame on a received start byte
   *
   * @param[in] start_byte MAVLINK_V1_STX or MAVLINK_V2_STX
   *
   * @return true if the byte was a start byte and a frame was started
   */
  bool start(uint8_t start_byte);

  /**
   * @brief Feed the next byte of a frame to the parser
   *
   * Returns true as soon as the checksum has been received, with the frame in
   * frame(). A v2 signature that follows is skipped by later calls.
   *
   * @param[in] byte Received byte
   *
   * @return true when a whole frame has been received
   */
  bool feed(uint8_t byte);

  /// Last frame received, valid once feed() returned true
  const MAVLinkFrame &frame() const { return this->frame_; }

 protected:
  /// Where the parser is within a frame
  enum State : uint8_t {
    STATE_IDLE = 0,   ///< Not inside a frame
    STATE_HEADER,     ///< Collecting the header after the start byte
    STATE_PAYLOAD,    ///< Collecting the payload
    STATE_CHECKSUM,   ///< Collecting the two checksum bytes
    STATE_SIGNATURE,  ///< Skipping the 13 byte v2 signature
  };

  /// Pull the fields out of a complete header
  void decode_header_();

  State state_{STATE_IDLE};
  MAVLinkFrame frame_{};
  uint8_t header_[9]{};        ///< Header bytes after the start byte
  uint8_t header_needed_{0};   ///< Header length, 5 for v1 and 9 for v2
  uint8_t header_got_{0};      ///< Header bytes collected so far
  uint8_t incompat_flags_{0};  ///< v2 incompatibility flags
  uint8_t payload_got_{0};     ///< Payload bytes collected so far
  uint16_t crc_running_{0};    ///< Checksum over everything after the start byte
  uint8_t crc_buf_[2]{};       ///< Checksum bytes as received
  uint8_t crc_got_{0};         ///< Checksum bytes collected so far
  uint8_t signature_got_{0};   ///< Signature bytes skipped so far
};

/**
 * @brief Decode a received MAVLink frame as ADSB_VEHICLE
 *
 * @param[in] frame Received frame
 * @param[out] out Decoded vehicle
 *
 * @return true on success, false if the frame is not an ADSB_VEHICLE message
 */
bool parse_mavlink_adsb_vehicle(const MAVLinkFrame &frame, ADSBVehicle &out);

}  // namespace esphome::soldered_adsb_receiver
