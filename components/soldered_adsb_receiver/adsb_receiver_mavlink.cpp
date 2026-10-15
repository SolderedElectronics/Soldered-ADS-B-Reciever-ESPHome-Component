/**
 * @file adsb_receiver_mavlink.cpp
 * @brief MAVLink frame parser and ADSB_VEHICLE decoder for the Aerobits TT-SC1b
 *
 * Framing follows the MAVLink v1 and v2 serialization described at
 * https://mavlink.io/en/guide/serialization.html
 *
 * @author Soldered Electronics
 */

#include "adsb_receiver_mavlink.h"

#include <cstring>

namespace esphome::soldered_adsb_receiver {

// Length of the ADSB_VEHICLE payload with no trailing zeros trimmed off
static const size_t ADSB_VEHICLE_PAYLOAD_LEN = 38;

// Length of the signature that may follow a v2 frame
static const uint8_t MAVLINK_SIGNATURE_LEN = 13;

// v2 incompatibility flag announcing a signature
static const uint8_t MAVLINK_IFLAG_SIGNED = 0x01;

// *****************************************************************************
// Section: Small helpers

/**
 * @brief Add one byte to a CRC-16/MCRF4XX checksum, as used by MAVLink
 *
 * @param[in] data Byte to add
 * @param[in,out] crc Running checksum
 */
static void crc_accumulate(uint8_t data, uint16_t &crc) {
  uint8_t tmp = data ^ (uint8_t) (crc & 0xFF);
  tmp ^= (uint8_t) (tmp << 4);
  crc = (uint16_t) ((crc >> 8) ^ ((uint16_t) tmp << 8) ^ ((uint16_t) tmp << 3) ^ (tmp >> 4));
}

/**
 * @brief Look up the CRC_EXTRA byte of a message
 *
 * MAVLink's checksum needs a per-message CRC_EXTRA byte that is not part of the
 * wire format, so only the messages listed here can have their checksum checked.
 *
 * @param[in] msgid Message id
 * @param[out] extra CRC_EXTRA byte of the message
 *
 * @return true if the message is known
 */
static bool crc_extra_for(uint32_t msgid, uint8_t &extra) {
  switch (msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:  // Sent once a second alongside ADSB_VEHICLE
      extra = 50;
      return true;
    case MAVLINK_MSG_ID_ADSB_VEHICLE:  // The aircraft data the module sends
      extra = 184;
      return true;
    default:
      return false;
  }
}

/**
 * @brief Read a little endian 16 bit value
 *
 * @param[in] p First byte
 *
 * @return Value read
 */
static uint16_t read_le16(const uint8_t *p) { return (uint16_t) p[0] | ((uint16_t) p[1] << 8); }

/**
 * @brief Read a little endian 32 bit value
 *
 * @param[in] p First byte
 *
 * @return Value read
 */
static uint32_t read_le32(const uint8_t *p) {
  return (uint32_t) p[0] | ((uint32_t) p[1] << 8) | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
}

// *****************************************************************************
// Section: Parser

void MAVLinkParser::decode_header_() {
  const uint8_t *header = this->header_;

  if (this->frame_.version == 1) {
    this->frame_.payload_len = header[0];
    this->frame_.seq = header[1];
    this->frame_.sysid = header[2];
    this->frame_.compid = header[3];
    this->frame_.msgid = header[4];
    this->incompat_flags_ = 0;
  } else {
    this->frame_.payload_len = header[0];
    this->incompat_flags_ = header[1];
    // header[2] holds the compat flags, which are not needed here
    this->frame_.seq = header[3];
    this->frame_.sysid = header[4];
    this->frame_.compid = header[5];
    // The message id is 3 bytes, little endian
    this->frame_.msgid = (uint32_t) header[6] | ((uint32_t) header[7] << 8) | ((uint32_t) header[8] << 16);
  }
}

bool MAVLinkParser::start(uint8_t start_byte) {
  if (start_byte != MAVLINK_V1_STX && start_byte != MAVLINK_V2_STX)
    return false;

  this->frame_.version = (start_byte == MAVLINK_V1_STX) ? 1 : 2;
  this->header_needed_ = (this->frame_.version == 1) ? 5 : 9;
  this->header_got_ = 0;
  this->crc_running_ = 0xFFFF;
  this->state_ = STATE_HEADER;

  return true;
}

bool MAVLinkParser::feed(uint8_t byte) {
  switch (this->state_) {
    case STATE_HEADER:
      this->header_[this->header_got_++] = byte;
      crc_accumulate(byte, this->crc_running_);
      if (this->header_got_ < this->header_needed_)
        break;

      this->decode_header_();
      this->payload_got_ = 0;
      this->crc_got_ = 0;
      this->state_ = (this->frame_.payload_len == 0) ? STATE_CHECKSUM : STATE_PAYLOAD;
      break;

    case STATE_PAYLOAD:
      this->frame_.payload[this->payload_got_++] = byte;
      crc_accumulate(byte, this->crc_running_);
      if (this->payload_got_ >= this->frame_.payload_len) {
        this->crc_got_ = 0;
        this->state_ = STATE_CHECKSUM;
      }
      break;

    case STATE_CHECKSUM: {
      this->crc_buf_[this->crc_got_++] = byte;
      if (this->crc_got_ < 2)
        break;

      uint16_t received = read_le16(this->crc_buf_);
      uint16_t computed = this->crc_running_;
      uint8_t extra;
      bool known_extra = crc_extra_for(this->frame_.msgid, extra);
      if (known_extra)
        crc_accumulate(extra, computed);
      this->frame_.crc_valid = known_extra && (computed == received);

      if (this->frame_.version == 2 && (this->incompat_flags_ & MAVLINK_IFLAG_SIGNED)) {
        this->signature_got_ = 0;
        this->state_ = STATE_SIGNATURE;
      } else {
        this->state_ = STATE_IDLE;
      }
      return true;
    }

    case STATE_SIGNATURE:
      if (++this->signature_got_ >= MAVLINK_SIGNATURE_LEN)
        this->state_ = STATE_IDLE;
      break;

    case STATE_IDLE:
    default:
      break;
  }

  return false;
}

// *****************************************************************************
// Section: Message decoders

bool parse_mavlink_adsb_vehicle(const MAVLinkFrame &frame, ADSBVehicle &out) {
  if (frame.msgid != MAVLINK_MSG_ID_ADSB_VEHICLE)
    return false;

  // MAVLink 2 lets the sender trim trailing zero bytes off the payload, so pad
  // it back up to the full layout before pulling the fields out
  uint8_t buf[ADSB_VEHICLE_PAYLOAD_LEN] = {0};
  size_t length = frame.payload_len < ADSB_VEHICLE_PAYLOAD_LEN ? frame.payload_len : ADSB_VEHICLE_PAYLOAD_LEN;
  memcpy(buf, frame.payload, length);

  memset(&out, 0, sizeof(out));
  out.icao_address = read_le32(&buf[0]);
  out.lat = (int32_t) read_le32(&buf[4]);
  out.lon = (int32_t) read_le32(&buf[8]);
  out.altitude = (int32_t) read_le32(&buf[12]);
  out.heading = read_le16(&buf[16]);
  out.hor_velocity = read_le16(&buf[18]);
  out.ver_velocity = (int16_t) read_le16(&buf[20]);
  out.flags = read_le16(&buf[22]);
  out.squawk = read_le16(&buf[24]);
  out.altitude_type = (AltitudeType) buf[26];
  memcpy(out.callsign, &buf[27], 8);
  out.callsign[8] = '\0';
  out.emitter_type = (EmitterType) buf[36];
  out.tslc = buf[37];

  return true;
}

}  // namespace esphome::soldered_adsb_receiver
