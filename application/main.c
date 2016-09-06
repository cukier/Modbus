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
#define START_W		15
#define QTD_W		120
#define END_PLC		64

int show_array(uint16_t *ptr, uint16_t size) {
	uint16_t cont;

	for (cont = 0; cont < size; ++cont) {
		if (!(cont % 8) && cont != 0)
			printf("\n");
		printf("0x%04X ", ptr[cont]);
	}
	printf("\n");

	return 0;
}

int gen_pattern(uint8_t *ptr, uint16_t size) {
	uint8_t cont, n;

//	cont = (uint8_t) rand() & 0xFF;
//	do {
//		n = (uint8_t) rand() & 0xFF;
//	} while (cont--);

	for (cont = 0; cont < size; ++cont) {
		ptr[cont] = (uint8_t) rand() & 0xFF;
	}

	return 0;
}

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

int read_plc() {
	int r;
	uint16_t cont, *to;

	to = NULL;
	to = (uint16_t *) malloc(QTD_R * sizeof(uint16_t));

	if (to == NULL)
		return -1;

	r = -1;
	r = read_holding_registers(END_PLC, START_R, QTD_R, to);

	if (r != 0) {
		free(to);
		fprintf(stderr, "Erro nr %u leitura\n", r);
		return -1;
	}

	show_array(to, QTD_R);
	free(to);

	return 0;
}

int write_plc() {
	uint16_t cont, *pattern;
	int r;

//		r = -1;
//		r = write_single_register(END_PLC, START_W, QTD_W);
//
//		if (r != 0) {
//			fprintf(stderr, "Erro nr %u escrita\n", r);
//			return -1;
//		}

	pattern = NULL;
	pattern = (uint16_t *) malloc(QTD_W * sizeof(uint16_t));

	if (pattern == NULL)
		return -1;

	gen_pattern((uint8_t *) pattern, 2 * QTD_W);
	printf("Padrao gerado\n");
	show_array(pattern, QTD_W);
	r = write_multiple_registers(END_PLC, START_W, QTD_W, (uint8_t *) pattern);
	free(pattern);

	return 0;
}

int main(int argc, char **argv) {

	char porta[] = "/dev/ttyS8";

	init(porta, 19200);

	printf("Porta aberta %s\nPeruntando...\n", porta);

//	read_plc();
	write_plc();
	serial_close();

	return 0;
}
