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

void show_array(uint16_t *ptr, uint16_t size) {
	uint16_t cont;

	for (cont = 0; cont < size; ++cont) {
		if (!(cont % 8) && cont != 0)
			printf("\n");
		printf("0x%04X ", ptr[cont]);
	}
	printf("\n");

	return;
}

int gen_pattern(uint8_t *ptr, uint16_t size) {
	uint16_t cont;

	for (cont = 0; cont < size; ++cont) {
		ptr[cont] = (uint8_t) rand() & 0xFF;
	}

	return 0;
}

exception_t init(char *m_porta, uint32_t baud_rate) {
	int r;

	r = -1;
	r = serial_init(m_porta, baud_rate);

	if (r == -1) {
		fprintf(stderr, "%s nao existe!\n", m_porta);
		return NO_SERIAL_PORT_EXCEPTION;
	}

	return 0;
}

exception_t read_plc() {
	exception_t r;
	uint16_t *to;

	to = NULL;
	to = (uint16_t *) malloc(QTD_R * sizeof(uint16_t));

	if (to == NULL)
		return -1;

	r = -1;
	r = mb_read_holding_registers(END_PLC, START_R, QTD_R, to);

	if (r != NO_EXCEPTION) {
		free(to);
		fprintf(stderr, "Erro nr %u leitura\n", r);
		return r;
	}

	show_array(to, QTD_R);
	free(to);

	return NO_EXCEPTION;
}

exception_t write_plc() {
	uint16_t *pattern;
	exception_t r;

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
		return OUT_OF_MEMORY_EXCEPTION;

	gen_pattern((uint8_t *) pattern, 2 * QTD_W);
	printf("Padrao gerado\n");
	show_array(pattern, QTD_W);
	r = mb_write_multiple_registers(END_PLC, START_W, QTD_W, pattern);
	free(pattern);

	return r;
}

int main(int argc, char **argv) {

	char porta[] = "/dev/ttyS9";
	char msg[20];
	exception_t ex;

	init(porta, 19200);

	printf("Porta aberta %s\nPeruntando...\n", porta);

	read_plc();
	ex = NO_EXCEPTION;
//	ex = write_plc();
	parse_error(msg, ex);
	printf("Escrevendo %s\n", msg);
	serial_close();

	return 0;
}
