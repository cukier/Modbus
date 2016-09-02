/*
 * main.c
 *
 *  Created on: 2 de set de 2016
 *      Author: cuki
 */

#include <stdio.h>
#include "modbus.h"
#include "serial.h"

int main(int argc, char **argv) {

	int fd;

	fd = serial_open_port("/dev/ttyS0");

	printf("Porta %u", fd);

	return 0;
}
