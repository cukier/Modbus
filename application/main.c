/*
 * main.c
 *
 *  Created on: 2 de set de 2016
 *      Author: cuki
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "modbus.h"
#include "serial.h"

#define MEM_SIZE	200
#define STEP		50
#define	START_R		0
#define QTD_R		12
#define START_W		0
#define QTD_W		10
#define END_PLC		64

int main(int argc, char **argv) {

	int r;
	char porta[] = "/dev/ttyS1";
	uint16_t *mem, cont;

	r = -1;
	r = serial_init(porta, 19200);

	if (r == -1) {
		fprintf(stderr, "%s nao existe!\n", porta);
		return -1;
	}

	printf("Porta aberta %s\nPeruntando...\n", porta);
	mem = NULL;
	mem = (uint16_t *) malloc(QTD_R * sizeof(uint16_t));

	if (mem == NULL) {
		fprintf(stderr, "Sem memoria!\n");
		return -1;
	}

	r = read_holding_registers(END_PLC, START_R, QTD_R, mem);

	if (r != 0) {
		fprintf(stderr, "Erro nr %u\n", r);
		return -1;
	}

	for (cont = 0; cont < QTD_R; ++cont) {
		if (!(cont % 8) && cont != 0)
			printf("\n");
		printf("0x%02X ", mem[cont]);
	}
	printf("\n");

	return 0;
}
