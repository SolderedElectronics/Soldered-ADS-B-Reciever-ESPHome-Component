/**
 * @file adsb_receiver_csv.h
 * @brief CSV protocol decoders for the Aerobits TT-SC1b
 *
 * These need no component instance, so they also work on logged data.
 *
 * @author Soldered Electronics
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include "adsb_receiver_defs.h"

namespace esphome::soldered_adsb_receiver {

/**
 * @brief Work out the CRC16 the module appends to its CSV lines
 *
 * @param[in] data Bytes to check
 * @param[in] length How many bytes there are
 *
 * @return CRC16, byte order as the module sends it
 */
uint16_t crc16(const uint8_t *data, size_t length);

/**
 * @brief Read and check the CRC field at the end of a CSV line
 *
 * @param[in] line CSV line starting with '#'
 * @param[out] computed CRC worked out over everything before the last comma
 * @param[out] received CRC sent in the last field
 *
 * @return true if the line has a CRC field; compare the two to check it
 */
bool verify_csv_crc(const char *line, uint16_t &computed, uint16_t &received);

/**
 * @brief Decode a "#A:" aircraft line
 *
 * @param[in] line Line as received
 * @param[out] out Decoded aircraft
 *
 * @return true on success, false if the line is not a whole "#A:" message
 */
bool parse_aircraft_line(const char *line, Aircraft &out);

/**
 * @brief Decode a "#AS:" ADS-B statistics line
 *
 * @param[in] line Line as received
 * @param[out] out Decoded statistics
 *
 * @return true on success, false if the line is not a whole "#AS:" message
 */
bool parse_adsb_stats_line(const char *line, ADSBStats &out);

/**
 * @brief Decode a "#S:" system statistics line
 *
 * @param[in] line Line as received
 * @param[out] out Decoded statistics
 *
 * @return true on success, false if the line is not a whole "#S:" message
 */
bool parse_system_stats_line(const char *line, SystemStats &out);

}  // namespace esphome::soldered_adsb_receiver
