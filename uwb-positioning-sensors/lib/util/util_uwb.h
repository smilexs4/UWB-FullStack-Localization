/**
 * @file util_uwb.h
 * @author smilexs4
 * @brief
 * @version 0.1
 * @date 2025-06-01
 *
 * @copyright Copyright (c) 2025 smilexs4
 * @license MIT License (see LICENSE.md for details)
 *
 */

#ifndef UTIL_UWB_H
#define UTIL_UWB_H

#include "dw3000_mac_802_15_4_short.h"
#include "util.h"
// #include "util_deca.h"
#include <Arduino.h>
// #include <dw3000.h>
#include "dw3000_deca_regs.h"
#include "dw3000_port_mod.h"
#include <ArduinoLog.h>
#include <SPI.h>

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion
 * factor. 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 63898

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
/* 2. The sum of the values is the TX to RX antenna delay, experimentally
 *determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay
 *properly calibrated to get the best possible precision when performing range
 *measurements. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#ifdef RPI_BUILD
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#endif // RPI_BUILD
#ifdef STM32F429xx
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#endif // STM32F429xx
#ifdef NRF52840_XXAA
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#endif // NRF52840_XXAA
/* Receive response timeout. See NOTE 5 below. */
#ifdef RPI_BUILD
#define RESP_RX_TIMEOUT_UUS 270
#endif // RPI_BUILD
#ifdef STM32F429xx
#define RESP_RX_TIMEOUT_UUS 210
#endif // STM32F429xx
#ifdef NRF52840_XXAA
#define RESP_RX_TIMEOUT_UUS 400
#endif // NRF52840_XXAA

#define POLL_TX_TO_RESP_RX_DLY_UUS 300
// #define RESP_RX_TIMEOUT_UUS 1500
// Indirect ranging initiator required a larger tmeut
#define RESP_RX_TIMEOUT_UUS 4000

#define CPU_PROCESSING_TIME 1200

// #define RESP_RX_TO_FINAL_TX_DLY_UUS (2000 + CPU_PROCESSING_TIME)
// #define POLL_RX_TO_RESP_TX_DLY_UUS (2000 + CPU_PROCESSING_TIME)
#define RESP_RX_TO_FINAL_TX_DLY_UUS (600 + CPU_PROCESSING_TIME)
#define POLL_RX_TO_RESP_TX_DLY_UUS (1000 + CPU_PROCESSING_TIME)

#define ADDR_SIZE_SHORT 2
#define ADDR_SIZE_EXTENDED 8

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and
 * power of the spectrum at the current temperature. These values can be
 * calibrated prior to taking reference measurements. See NOTE 5 below. */
// extern dwt_txconfig_t txconfig_options;

bool uwb_init(uint32_t bitmask_lo, dwt_cb_t cbTxDone, dwt_cb_t cbRxOk,
              dwt_cb_t cbRxTo, dwt_cb_t cbRxErr);

void init_initiator(void);
void init_responder(void);
uint64_t get_eui64(void);
uint16_t get_addr_self(void);

void init_initiator_v2(void);

bool mac_parse(uint8_t *buf, uint16_t len, mac_frame_802_15_4_format_t *frame,
               uint32_t *payload_len, boolean allow_empty_payload);
void mac_prepare_frame(mac_frame_802_15_4_format_t *frame, uint8_t seq_num,
                       uint64_t dest_eui64, uint64_t src_eui64,
                       void *payload_ptr, uint16_t pan_id);
bool mac_send_frame(mac_frame_802_15_4_format_t *frame, size_t payload_len,
                    uint8_t mode);
bool mac_read_frame(mac_frame_802_15_4_format_t *frame, uint8_t *buf,
                    uint32_t *payload_len, uint32_t max_len);

#endif // UTIL_UWB_H