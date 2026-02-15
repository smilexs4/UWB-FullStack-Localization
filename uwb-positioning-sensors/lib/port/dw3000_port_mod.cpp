/**
 * @file dw3000_port_mod.cpp
 * @author smilexs4
 * @brief
 * @version 0.1
 * @date 2025-06-19
 *
 * @copyright Copyright (c) 2025 smilexs4
 * @license MIT License (see LICENSE.md for details)
 *
 */

#include "dw3000_port_mod.h"

void deca_usleep(uint16_t time_us) { sleepus(time_us); }

void deca_usleep(uint32_t time_us) { sleepus(time_us); }

void deca_sleep(unsigned int time_ms) { sleepms(time_ms); }

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexon()
 *
 * Description: This function should disable interrupts. This is called at the start of a critical section
 * It returns the irq state before disable, this value is used to re-enable in decamutexoff call
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
// #ifndef NRF52_SERIES
// portMUX_TYPE my_mutex = portMUX_INITIALIZER_UNLOCKED;
// #endif
// decaIrqStatus_t decamutexon(void)
// {
// #ifdef NRF52_SERIES
//     portENTER_CRITICAL();
// #else
//     portENTER_CRITICAL(&my_mutex);
// #endif
//     portDISABLE_INTERRUPTS();
//     decaIrqStatus_t s = port_GetEXT_IRQStatus();

//     if(s) {
//         port_DisableEXT_IRQ(); //disable the external interrupt line
//     }
//     return s ;   // return state before disable, value is used to re-enable in decamutexoff call
// }

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexoff()
 *
 * Description: This function should re-enable interrupts, or at least restore their state as returned(&saved) by decamutexon
 * This is called at the end of a critical section
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 * @param s - the state of the DW1000 interrupt as returned by decamutexon
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
// void decamutexoff(decaIrqStatus_t s)        // put a function here that re-enables the interrupt at the end of the critical section
// {
// #ifdef NRF52_SERIES
//     portEXIT_CRITICAL();
// #else
//     portEXIT_CRITICAL(&my_mutex);
// #endif
//     portENABLE_INTERRUPTS();

//     if(s) { //need to check the port state as we can't use level sensitive interrupt on the STM ARM
//         port_EnableEXT_IRQ();
//     }
// }



// Global mutex for DW3000 access
static SemaphoreHandle_t dw3000_mutex = NULL;

/**
 * @brief Initialize the decamutex system
 * 
 * This should be called once during system initialization
 */
int decamutex_init(void) {
    if (dw3000_mutex == NULL) {
        dw3000_mutex = xSemaphoreCreateMutex();
        if (dw3000_mutex == NULL) {
            return -1;
        }
    }
    return 0;
}

/**
 * @brief Acquire the DW3000 mutex
 * 
 * @return decaIrqStatus_t Previous interrupt state (not used on ESP32)
 */
decaIrqStatus_t decamutexon(void) {
    // if (xSemaphoreTake(dw3000_mutex, portMAX_DELAY) != pdTRUE) {
    //     // ESP_LOGE("DECAMUTEX", "Failed to acquire mutex");
    // }
    
    // // On ESP32, we don't need to return the interrupt state
    // return 0;

    return xSemaphoreTake(dw3000_mutex, portMAX_DELAY) == pdTRUE;
}

/**
 * @brief Release the DW3000 mutex
 * 
 * @param s Previous interrupt state (ignored on ESP32)
 */
void decamutexoff(decaIrqStatus_t s) {
    (void)s; // Unused parameter
    
    if (dw3000_mutex == NULL) {
        // ESP_LOGE("DECAMUTEX", "Mutex not initialized");
        return;
    }
    
    xSemaphoreGive(dw3000_mutex);
}

decaIrqStatus_t decamutexon_isr() {
  return xSemaphoreGiveFromISR(dw3000_mutex, NULL) == pdTRUE;
}