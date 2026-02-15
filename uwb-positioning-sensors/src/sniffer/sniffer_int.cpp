/**
 * @file sniffer_int.cpp
 * @author smilexs4
 * @brief UWB Packet sniffer for IEEE 802.15.4 Serial Interface
 * @version 0.1
 * @date 2025-06-24
 *
 * @copyright Copyright (c) 2025 smilexs4
 * @license MIT License (see LICENSE.md for details)
 *
 */

#include "util_uwb.h"

#ifdef NRF52_SERIES
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#endif

/*
 * 802.15.4 Serial Interace frame format
 * +--------+------------------------------------------------+
 * | Length |                   Payload                      |
 * | 1 byte |               Up to 256 bytes                  |
 * +--------+------------------------------------------------+
 */
uint8_t rx_buffer[1 + 256];

static void rxOk(const dwt_cb_data_t *cb_data) {
  // decamutex already acquired by caller function, dwt calls are safe here

  if (cb_data->datalength > 256) {
    // Do nothing, frame is too large
    // This can happen if extended frame size is used
    // i.e. DWT_PHRMODE_EXT allows frames of length 5 to 1023 octets long
    return;
  }

  // First byte is the payload length
  rx_buffer[0] = cb_data->datalength;

  // Read the payload from chip buffer
  dwt_readrxdata(&rx_buffer[1], cb_data->datalength, 0);

  // Write frame to serial
  Serial.write(rx_buffer, cb_data->datalength + 1);

  // Reenable receiver
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void setup() {
  Serial.begin(115200);

  // Wait for Serial to attach (on NRF52)
  while (!Serial)
    delay(100);

  // Initialize UWB library and peripheral
  uwb_init(DWT_INT_RXFCG_BIT_MASK, NULL, rxOk, NULL, NULL);

  // Disable timeout
  dwt_setrxtimeout(0);

  // Start receiver
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void loop() {}