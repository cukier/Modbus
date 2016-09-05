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
#define QTD_R		120
#define START_W		0
#define QTD_W		10
#define END_PLC		64

int init(char *m_porta, uint32_t baud_rate) {
	int r;

	r = -1;
	r = serial_init(m_porta, baud_rate);

	if (r == -1) {
		fprintf(stderr, "%s nao existe!\n", m_porta);
		return -1;
	}

	return 0;
}

int read_plc(uint16_t *to) {
	int r;
	uint16_t cont;

	r = -1;
	r = read_holding_registers(END_PLC, START_R, QTD_R, to);

	if (r != 0) {
		fprintf(stderr, "Erro nr %u leitura\n", r);
		return -1;
	}

	for (cont = 0; cont < QTD_R; ++cont) {
		if (!(cont % 8) && cont != 0)
			printf("\n");
		printf("0x%04X ", to[cont]);
	}
	printf("\n");

	return 0;
}

int write_plc(uint16_t *from, size_t f_size) {
	int r;

	r = -1;
	r = write_single_register(END_PLC, START_W, QTD_W);

	if (r != 0) {
		fprintf(stderr, "Erro nr %u escrita\n", r);
		return -1;
	}

	return 0;
}

int main(int argc, char **argv) {

	char porta[] = "/dev/ttyS9";
	uint16_t *mem;

	init(porta, 19200);

	printf("Porta aberta %s\nPeruntando...\n", porta);
	mem = NULL;
	mem = (uint16_t *) malloc(QTD_R * sizeof(uint16_t));

	if (mem == NULL) {
		fprintf(stderr, "Sem memoria!\n");
		return -1;
	}

	read_plc(mem);
	write_plc(NULL, 0);
	serial_close();

//	r = -1;
//	r = write_single_register(END_PLC, 0, 1);

	return 0;
}
