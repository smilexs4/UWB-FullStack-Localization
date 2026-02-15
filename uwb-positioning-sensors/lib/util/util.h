/**
 * @file util.h
 * @author smilexs4
 * @brief
 * @version 0.1
 * @date 2025-06-01
 *
 * @copyright Copyright (c) 2025 smilexs4
 * @license MIT License (see LICENSE.md for details)
 *
 */

#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>
#include <ArduinoLog.h>

char *get_eui64_string(uint64_t eui64);
void print_mac_address(uint64_t mac);
void print_eui64(uint64_t eui64);
uint64_t mac_to_eui64(uint64_t mac48);
uint64_t eui64_to_uint64(const uint8_t eui[8]);
void print_addr_16(const uint16_t addr);

#endif // UTIL_H