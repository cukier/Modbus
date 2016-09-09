/*
 * serial.h
 *
 *  Created on: 07/07/2015
 *      Author: cuki
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

extern int serial_init(char p[], uint32_t b);
extern int serial_close(void);
extern size_t serial_transaction(uint8_t *tx, uint8_t *rx, uint16_t msg_size,
		uint16_t resp_size);

#endif /* SERIAL_H_ */
