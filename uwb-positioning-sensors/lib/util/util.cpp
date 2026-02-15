/**
 * @file util.cpp
 * @author smilexs4
 * @brief
 * @version 0.1
 * @date 2025-06-01
 *
 * @copyright Copyright (c) 2025 smilexs4
 * @license MIT License (see LICENSE.md for details)
 *
 */
#include "util.h"

char *get_eui64_string(uint64_t eui64) {
  char *buffer = (char *)malloc(24);
  if (buffer == NULL) {
    return NULL;
  }
  snprintf(buffer, 24, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
           (uint8_t)(eui64 >> 56), (uint8_t)(eui64 >> 48),
           (uint8_t)(eui64 >> 40), (uint8_t)(eui64 >> 32),
           (uint8_t)(eui64 >> 24), (uint8_t)(eui64 >> 16),
           (uint8_t)(eui64 >> 8), (uint8_t)(eui64));
  return buffer;
}

void print_mac_address(uint64_t mac) {
  Serial.printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                (uint8_t)(mac >> 40) & 0xFF, (uint8_t)(mac >> 32) & 0xFF,
                (uint8_t)(mac >> 24) & 0xFF, (uint8_t)(mac >> 16) & 0xFF,
                (uint8_t)(mac >> 8) & 0xFF, (uint8_t)(mac) & 0xFF);
}

void print_eui64(uint64_t eui64) {
  char *eui_str = get_eui64_string(eui64);
  Serial.printf("EUI-64: %s\r\n", eui_str);
  free(eui_str);
}

uint64_t mac_to_eui64(uint64_t mac48) {
  // Extract each MAC byte
  uint8_t mac[6];
  for (int i = 0; i < 6; i++) {
    mac[5 - i] = (mac48 >> (i * 8)) & 0xFF;
  }

  // Build EUI-64
  uint64_t eui64 = 0;
  eui64 |= (uint64_t)(mac[0] ^ 0x02) << 56; // Flip U/L bit
  eui64 |= (uint64_t)mac[1] << 48;
  eui64 |= (uint64_t)mac[2] << 40;
  eui64 |= (uint64_t)0xFF << 32;
  eui64 |= (uint64_t)0xFE << 24;
  eui64 |= (uint64_t)mac[3] << 16;
  eui64 |= (uint64_t)mac[4] << 8;
  eui64 |= (uint64_t)mac[5];

  return eui64;
}

uint64_t eui64_to_uint64(const uint8_t eui[8]) {
  uint64_t result = 0;
  for (int i = 0; i < 8; i++) {
    result = (result << 8) | eui[i];
  }
  return result;
}

void print_addr_16(const uint16_t addr) { Log.notice("%X", addr); }