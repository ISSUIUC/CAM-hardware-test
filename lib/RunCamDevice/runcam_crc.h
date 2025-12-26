#pragma once
#include <Arduino.h>
constexpr uint8_t rc_crc8_dvb_s2(uint8_t crc, unsigned char a) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }

    return crc;
}

constexpr uint8_t rc_generate_crc(const uint8_t* buf, unsigned int buf_len) {
  uint8_t crc = 0x00;
  for(unsigned i = 0; i < buf_len; i++) {
    crc = rc_crc8_dvb_s2(crc, buf[i]);
  }
  return crc;
}

constexpr bool rc_check_crc(uint8_t* buf, unsigned int buf_len, uint8_t expected_crc) {
  return rc_generate_crc(buf, buf_len) == expected_crc;
}