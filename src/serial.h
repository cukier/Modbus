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

extern int serial_open_port(char *porta);
extern int serial_close_port(int fd);
extern int serial_set_port(int baud_rate, int fd);
extern int serial_transaction(int fd, uint8_t *msg, uint8_t *resp, int msg_size,
		int resp_size);

#endif /* SERIAL_H_ */
