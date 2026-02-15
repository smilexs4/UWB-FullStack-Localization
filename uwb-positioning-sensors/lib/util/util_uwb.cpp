/**
 * @file util_uwb.cpp
 * @author smilexs4
 * @brief
 * @version 0.1
 * @date 2025-06-01
 *
 * @copyright Copyright (c) 2025 smilexs4
 * @license MIT License (see LICENSE.md for details)
 *
 */

#include "util_uwb.h"

// DW3xxx API Guide, Version 4.7, Page 40 of 185
static dwt_txconfig_t txconfig_options = {0x34, 0xfdfdfdfd, 0x0};

extern struct dwt_driver_s dw3000_driver;

dwt_spi_s spi = {.readfromspi = readfromspi,
                 .writetospi = writetospi,
                 .writetospiwithcrc = nullptr,
                 .setslowrate = nullptr,
                 .setfastrate = nullptr};

// dwchip_s chip = {.SPI = &spi, .dwt_driver = &dw3000_driver};
static dwchip_s chip;

static struct dwt_driver_s *my_driver_list[] = {&dw3000_driver};

static struct dwt_driver_s **driver_list = my_driver_list;

static dwt_probe_s dw3000_probe_interf{
    .dw = &chip,
    .spi = &spi,
    .wakeup_device_with_io = [] {}, // wakeup_device_with_io,
    .driver_list = driver_list,
    .dw_driver_num = 1,
};

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    DWT_SFD_DW_8,    /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8
             symbol, 2 for    non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF
             type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size).
                      Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e
                       */
    DWT_PDOA_M0       /* PDOA mode off */
};

static uint8_t rx_buffer[1024];
dwt_callbacks_s callbacks;

static void init_uwb_periph(void);

// Declare a binary semaphore to signal the task
SemaphoreHandle_t dwt_isr_semaphore;

void task_dwt(void *pvParams) {
  while (true) {
    // Wait indefinitely for the ISR to signal
    if (xSemaphoreTake(dwt_isr_semaphore, portMAX_DELAY) == pdTRUE) {
      // Now that the ISR has signaled, acquire the mutex to access DW3000
      decaIrqStatus_t mutex = decamutexon();
      if (mutex) {
        do {
          dwt_isr(); // ISR
        } while (dwt_checkirq()); // Run until IRQ line is active
        decamutexoff(mutex); // Release the mutex
      } else {
        // This should ideally not happen if mutex is managed correctly
      }
    }
  }
}

#ifdef ESP32
// ESP32 requires IRAM_ATTR for INT handler function signature
void IRAM_ATTR isr_dwt()
#else
void isr_dwt()
#endif
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Signal the task that an interrupt occurred
  xSemaphoreGiveFromISR(dwt_isr_semaphore, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken == pdTRUE) {
#ifdef ESP32
    portYIELD_FROM_ISR(); // Request a context switch if a higher priority task
                          // was unblocked
#else
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // nRF52 requires an argument
#endif
  }
}

bool uwb_init(uint32_t bitmask_lo, dwt_cb_t cbTxDone, dwt_cb_t cbRxOk,
              dwt_cb_t cbRxTo, dwt_cb_t cbRxErr) {
  int32_t res;

  dwt_isr_semaphore = xSemaphoreCreateBinary();
  if (dwt_isr_semaphore == NULL) {
    Serial.println("[UWB] Failed to create ISR Semaphore");
  }

  int res_deca_mutex = decamutex_init();
  if (res_deca_mutex != 0) {
    Serial.println("[UWB] Failed to create decamutex");
    return false;
  }

  init_uwb_periph();

  res = dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);
  if (res != DWT_SUCCESS) {
    Serial.printf("[UWB] dwt_probe failed with error code: %d\n", res);
    return false;
  }

  res = dwt_initialise(DWT_READ_OTP_PID | DWT_READ_OTP_LID | DWT_READ_OTP_BAT |
                       DWT_READ_OTP_TMP);
  if (res != DWT_SUCCESS) {
    Serial.printf("[UWB] dwt_initialise failed with error code: %d\n", res);
    return false;
  }

  while (!dwt_checkidlerc()) {
  };

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    Serial.println("INIT FAILED     ");
    while (1) {
    };
  }

  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  callbacks.cbTxDone = cbTxDone;
  callbacks.cbRxOk = cbRxOk;
  callbacks.cbRxTo = cbRxTo;
  callbacks.cbRxErr = cbRxErr;
  dwt_setcallbacks(&callbacks);

  // dwt_setinterrupt(0x00004000, 1, DWT_ENABLE_INT);
  /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and
   * RX errors). */
  // dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK |
  //                      DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK |
  //                      DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK |
  //                      DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK,
  //                  0, DWT_ENABLE_INT);
  dwt_setinterrupt(bitmask_lo, 0, DWT_ENABLE_INT);
  /*Clearing the SPI ready interrupt*/
  dwt_writesysstatuslo(DWT_INT_RCINIT_BIT_MASK | DWT_INT_SPIRDY_BIT_MASK);

  // port_set_dwic_isr(dwt_isr);

  // Attach plarform specific hardware interrupts to the corresponding pins
#ifdef ESP32
  attachInterrupt(digitalPinToInterrupt(4), isr_dwt, RISING);
#else
  attachInterrupt(digitalPinToInterrupt(34), isr_dwt, RISING);
#endif

  // Create platform specific RTOS tasks
#ifdef ESP32
  // ESP32 allows running tasks on a specific core
  xTaskCreatePinnedToCore(task_dwt, "Interrupt Handler", 8192, NULL, 20, NULL,
                          1);
#else
  xTaskCreate(task_dwt,            // Function that implements the task
              "Interrupt Handler", // Text name for the task
              8192,                // Stack size in words
              NULL,                // No parameter passed into the task
              20,                  // High priority
              NULL                 // Task handle not required
  );
#endif

  res = dwt_configure(&config);
  if (res != DWT_SUCCESS) {
    Serial.printf("[UWB] dwt_configure failed with error code: %d\n", res);
    return false;
  }

  dwt_configuretxrf(&txconfig_options);

  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  return true;
}

void init_initiator(void) {
  init_uwb_periph();

  delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to
            // IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before
                             // proceeding
  {
    Log.fatal("IDLE FAILED\r\n");
    while (1)
      ;
  }
  Log.trace("IDLE OK\r\n");

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    Log.fatal("INIT FAILED\r\n");
    while (1)
      ;
  }
  Log.trace("INIT OK\r\n");

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on
  // DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 6 below. */
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either
                              // the PLL or RX calibration has failed the host
                              // should reset the device
  {
    Log.fatal("CONFIG FAILED\r\n");
    while (1)
      ;
  }
  Log.trace("CONFIG OK\r\n");

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
   * As this example only handles one incoming frame with always the same delay
   * and timeout, those values can be set here once for all. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and
   * also TX/RX LEDs Note, in real low power applications the LEDs should not be
   * used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  Log.trace("INIT DONE\r\n");
}

void init_responder(void) {
  init_uwb_periph();

  delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to
            // IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before
                             // proceeding
  {
    Log.fatal("IDLE FAILED" CRLF);
    while (1)
      ;
  }
  Log.trace("IDLE OK" CRLF);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    Log.fatal("INIT FAILED" CRLF);
    while (1)
      ;
  }
  Log.trace("INIT OK" CRLF);

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on
  // DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 6 below. */
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either
                              // the PLL or RX calibration has failed the host
                              // should reset the device
  {
    Log.fatal("CONFIG FAILED" CRLF);
    while (1)
      ;
  }
  Log.trace("CONFIG OK" CRLF);

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and
   * also TX/RX LEDs Note, in real low power applications the LEDs should not be
   * used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  Log.trace("INIT DONE");
}

static void init_uwb_periph(void) {
#ifdef NRF52_SERIES
  SPI.setPins(29, 3, 8);

  /* Configure SPI rate, DW3000 supports up to 38 MHz */
  /* Reset DW IC */
  spiBegin(34, 25);
  // spiSelect(38);
  // return;

  reselect(38);
  // try locking clock at PLL speed (should be done already,
  // but just to be sure)
  // enableClock(AUTO_CLOCK);
  delay(5);
  pinMode(25, OUTPUT);
  digitalWrite(25, LOW);
  delay(2); // dw1000 data sheet v2.08 §5.6.1 page 20: nominal 50ns, to be safe
            // take more time
  pinMode(25, INPUT);
  delay(10); // dwm1000 data sheet v1.2 page 5: nominal 3 ms, to be safe take
             // more time
  // force into idle mode (although it should be already after reset)
  // idle();

#elif defined(ESP32)
  spiBegin(4, 15);
  // spiSelect(5);
  // return;

  reselect(5);
  // try locking clock at PLL speed (should be done already,
  // but just to be sure)
  // enableClock(AUTO_CLOCK);
  delay(5);
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  delay(2); // dw1000 data sheet v2.08 §5.6.1 page 20: nominal 50ns, to be safe
            // take more time
  pinMode(15, INPUT);
  delay(10); // dwm1000 data sheet v1.2 page 5: nominal 3 ms, to be safe take
             // more time
  // force into idle mode (although it should be already after reset)
  // idle();

#else
#error "Unsupported platform for UWB peripheral initialization"
#endif
}

/**
 * @brief Generate EUI-64 from the device's part ID and lot ID. In production
 * this should be read from OTP memory.
 *
 * @return uint64_t
 */
uint64_t get_eui64(void) {
  uint32_t part_id = dwt_getpartid();
  uint32_t lot_id = dwt_getlotid();

  uint64_t my_mac = ((uint64_t)lot_id << 32) | part_id;
  print_mac_address(my_mac);

  uint64_t eui64 = mac_to_eui64(my_mac);

#if ADDR_SIZE == ADDR_SIZE_SHORT
  eui64 &= 0xFFFF;
#endif

  print_eui64(eui64);

  return eui64;
}

uint16_t get_addr_self(void) {
  uint32_t part_id = dwt_getpartid();
  return (uint16_t)(part_id & 0xFFFF);
}

void mac_prepare_frame(mac_frame_802_15_4_format_t *frame, uint8_t seq_num,
                       uint64_t dest_eui64, uint64_t src_eui64,
                       void *payload_ptr, uint16_t pan_id) {
  // Initialize MAC frame control fields

  // Configure Auxiliary Security Control Field
#if ADDR_SIZE == ADDR_SIZE_EXTENDED
  MAC_FRAME_AUX_SECURITY_CTRL_802_15_4(frame) =
      (AUX_SEC_LEVEL_DATA_CONF_OFF_MIC_0 << AUX_SECURITY_LEVEL_SHIFT_VALUE) |
      (AUX_KEY_IDEN_MODE_KEY_INDEX << AUX_KEY_IDENTIFIER_MODE_SHIFT_VALUE) |
      (AUX_FRAME_CNT_SUPPRESS_OFF << AUX_FRAME_CNT_SUPPRESSION_SHIFT_VALUE) |
      (AUX_ASN_IN_NOUNCE_FRAME_CNT_NOT_GEN_NONCE << AUX_ASN_IN_NONCE);
  MAC_FRAME_FRAME_CTRL_802_15_4(frame, 0) =
      (MAC_FRAME_TYPE_802_15_4_DATA << MAC_FRAME_TYPE_SHIFT_VALUE) |
      (MAC_SEC_EN_UN_PROTECTED << MAC_FRAME_SECURITY_ENABLED_SHIFT_VALUE) |
      (MAC_PEND_FRAME_NO_MORE_DATA << MAC_FRAME_FRAME_PENDING_SHIFT_VALUE) |
      (MAC_AR_NO_ACK << MAC_FRAME_AR_SHIFT_VALUE) |
      (MAC_PEND_ID_COMPRESS_DEST_EXIST_SOURCE_NOT
       << MAC_FRAME_PAN_ID_SHIFT_VALUE);
  MAC_FRAME_FRAME_CTRL_802_15_4(frame, 1) =
      (MAC_SEQ_NUM_SUPP_PRESENT << MAC_FRAME_SEQ_NUM_SUPPRESS_SHIFT_VALUE) |
      (MAC_SEQ_NUM_SUPP_PRESENT << MAC_FRAME_IE_PRESET_SHIFT_VALUE) |
      (MAC_DEST_ADDR_MODE_EXT_ADDR_64_BITS
       << MAC_FRAME_DEST_ADDR_MODE_SHIFT_VALUE) |
      (DATA_FRAME_VERSION << MAC_FRAME_FRAME_VER_SHIFT_VALUE) |
      (MAC_SRC_ADDR_MODE_EXT_ADDR_64_BITS
       << MAC_FRAME_SRC_ADDR_MODE_SHIFT_VALUE);
#else
  MAC_FRAME_FRAME_CTRL_802_15_4(frame, 0) =
      (MAC_FRAME_TYPE_802_15_4_DATA << MAC_FRAME_TYPE_SHIFT_VALUE) |
      (MAC_SEC_EN_UN_PROTECTED << MAC_FRAME_SECURITY_ENABLED_SHIFT_VALUE) |
      (MAC_PEND_FRAME_NO_MORE_DATA << MAC_FRAME_FRAME_PENDING_SHIFT_VALUE) |
      (MAC_AR_NO_ACK << MAC_FRAME_AR_SHIFT_VALUE) |
      (MAC_PEND_ID_COMPRESS_DEST_EXIST_SOURCE_NOT
       << MAC_FRAME_PAN_ID_SHIFT_VALUE);
  MAC_FRAME_FRAME_CTRL_802_15_4(frame, 1) =
      (MAC_SEQ_NUM_SUPP_PRESENT << MAC_FRAME_SEQ_NUM_SUPPRESS_SHIFT_VALUE) |
      (MAC_SEQ_NUM_SUPP_PRESENT << MAC_FRAME_IE_PRESET_SHIFT_VALUE) |
      (MAC_DEST_ADDR_MODE_SHORT_ADDR_16_BITS
       << MAC_FRAME_DEST_ADDR_MODE_SHIFT_VALUE) |
      (DATA_FRAME_VERSION << MAC_FRAME_FRAME_VER_SHIFT_VALUE) |
      (MAC_SRC_ADDR_MODE_SHORT_ADDR_16_BITS
       << MAC_FRAME_SRC_ADDR_MODE_SHIFT_VALUE);
#endif

  // Set sequence number
  mac_frame_update_sequence_number_short(frame, seq_num);

  // Set PAN ID and addresses
  mac_frame_set_pan_ids_and_addresses_802_15_4_short(frame, pan_id, dest_eui64,
                                                     pan_id, src_eui64);

  // Set payload pointer
  PAYLOAD_PTR_802_15_4(frame) = (uint8_t *)payload_ptr;
}

bool mac_send_frame(mac_frame_802_15_4_format_t *frame, size_t payload_len,
                    uint8_t mode) {
  // Calculate total frame size
  uint16_t total_frame_size = sizeof(mhr_802_15_4_t) + payload_len;
  // Prepare transmission
  // dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  dwt_write_reg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  // Write MAC header
  dwt_writetxdata(sizeof(mhr_802_15_4_t), (uint8_t *)&frame->mhr_802_15_4, 0);
  // Write payload
  dwt_writetxdata(payload_len, frame->payload_ptr, sizeof(mhr_802_15_4_t));
  // The last two bytes of the frame are FCS populated by the DW IC
  dwt_writetxfctrl(total_frame_size + 2, 0, 1);
  // Start transmission with the provided mode
  return dwt_starttx(mode) == DWT_SUCCESS;
}

bool mac_parse(uint8_t *buf, uint16_t len, mac_frame_802_15_4_format_t *frame,
               uint32_t *payload_len, boolean allow_empty_payload) {
  // Exclude FCS length (2 bytes) from received data length
  uint16_t len_payload_in = len - 2;

  // Check incoming payload length
  if (len_payload_in < sizeof(mhr_802_15_4_t)) {
    // Insufficient bytes
    return false;
  } else if (!allow_empty_payload && len_payload_in == sizeof(mhr_802_15_4_t)) {
    // Empty payload
    return false;
  }

  // Parse MAC header
  memcpy(&frame->mhr_802_15_4, buf, sizeof(mhr_802_15_4_t));

  // Set MAC payload length and pointer
  *payload_len = len_payload_in - sizeof(mhr_802_15_4_t);
  PAYLOAD_PTR_802_15_4(frame) = buf + sizeof(mhr_802_15_4_t);

  return true;
}

bool mac_read_frame(mac_frame_802_15_4_format_t *frame, uint8_t *buf,
                    uint32_t *payload_len, uint32_t max_len) {
  // Read frame length
  // uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  uint32_t frame_len = dwt_read_reg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;

  // Check if payload buffer is large enough
  if (frame_len > max_len) {
    return false;
  }

  // Check if frame length is sufficient for MAC header
  // The last two bytes of the frame are FCS populated by the DW IC
  if (frame_len < sizeof(mhr_802_15_4_t) + 2) {
    return false;
  }

  // Read frame into the local buffer
  dwt_readrxdata(buf, frame_len, 0);

  // Parse MAC frame
  memcpy(&frame->mhr_802_15_4, buf, sizeof(mhr_802_15_4_t));

  *payload_len = 0;
  PAYLOAD_PTR_802_15_4(frame) = NULL;

  // Set payload pointer based on frame length
  // The last two bytes of the frame are FCS populated by the DW IC
  if (frame_len > sizeof(mhr_802_15_4_t) + 2) {
    *payload_len = frame_len - sizeof(mhr_802_15_4_t) - 2;
    PAYLOAD_PTR_802_15_4(frame) = buf + sizeof(mhr_802_15_4_t);
  }

  return true;
}

void init_initiator_v2() {
  /* Configure SPI rate, DW3000 supports up to 38 MHz */
  // port_set_dw_ic_spi_fastrate(4, 15, 5);
  // Change pins as needed
  Log.trace("SPI begin" CRLF);
  spiBegin(4, 15);
  spiSelect(5);
  Log.trace("SPI end" CRLF);
  // Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to
  // IDLE_RC

  // softReset();
  // reset_DWIC();
  // reset();

  delay(2);

  // dwt_update_dw(&chip);

  /* Probe for the correct device driver. */
  dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

  if (dwt_initialise(DWT_READ_OTP_PID | DWT_READ_OTP_LID | DWT_READ_OTP_BAT |
                     DWT_READ_OTP_TMP) == DWT_ERROR) {
    Log.fatal("INIT FAILED     ");
    while (1) {
    };
  }

  while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before
                                proceeding */
  {
  };

  /* Configure DW IC. See NOTE 14 below. */
  /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has
   * failed the host should reset the device */
  if (dwt_configure(&config)) {
    Log.fatal("CONFIG FAILED     ");
    while (1) {
    };
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Apply default antenna delay value. See NOTE 1 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Set expected response's delay and timeout. See NOTE 4, 5 and 7 below.
   * As this example only handles one incoming frame with always the same delay
   * and timeout, those values can be set here once for all. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
  dwt_setpreambledetecttimeout(5);

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and
   * also TX/RX LEDs Note, in real low power applications the LEDs should not be
   * used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
}