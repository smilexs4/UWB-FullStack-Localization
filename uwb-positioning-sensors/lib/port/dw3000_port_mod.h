/**
 * @file dw3000_port_mod.h
 * @author smilexs4
 * @brief
 * @version 0.1
 * @date 2025-06-19
 *
 * @copyright Copyright (c) 2025 smilexs4
 * @license MIT License (see LICENSE.md for details)
 *
 */

#ifndef PORT_MOD_H_
#define PORT_MOD_H_

#include "deca_interface.h"
#include "dw3000_port.h"

void deca_usleep(uint16_t time_us);
void deca_usleep(uint32_t time_us);

void deca_sleep(unsigned int time_ms);

int decamutex_init(void);

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
decaIrqStatus_t decamutexon(void);

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
void decamutexoff(decaIrqStatus_t s);

decaIrqStatus_t decamutexon_isr();

#endif