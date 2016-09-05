/*
 * modbus.c
 *
 *  Created on: 20/08/2015
 *      Author: cuki
 *
 *
 *
 */

#include "modbus.h"
#include "serial.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

uint16_t make16(uint8_t varhigh, uint8_t varlow) {
	return (uint16_t) (varhigh & 0xff) * 0x100 + (varlow & 0xff);
}

uint8_t make8(uint32_t var, uint8_t offset) {
	return (uint8_t) (((var >> (offset * 8)) & 0xff));
}

uint16_t CRC16(uint8_t *nData, uint16_t wLength) {
	uint8_t nTemp;
	uint16_t wCRCWord = 0xFFFF;

	while (wLength--) {
		nTemp = *nData++ ^ wCRCWord;
		wCRCWord >>= 8;
		wCRCWord ^= wCRCTable[nTemp];
	}

	return wCRCWord;
}

uint8_t make_request(uint8_t dev_addr, uint16_t from, uint16_t size,
		uint8_t byte_count, uint8_t *data, modbus_command_t command,
		uint8_t *req) {
	uint16_t crc, cont;

	req[0] = dev_addr;
	req[1] = (uint8_t) command;
	req[2] = make8(from, 1);
	req[3] = make8(from, 0);
	req[4] = make8(size, 1);
	req[5] = make8(size, 0);

	switch (command) {
	case READ_COILS_COMMAND:
	case READ_DISCRETE_INPUT_COMMAND:
	case READ_HOLDING_REGISTERS_COMMAND:
		crc = CRC16(req, 6);
		req[6] = make8(crc, 0);
		req[7] = make8(crc, 1);
		break;
	case WRITE_MULTIPLE_REGISTERS_COMMAND:
		req[6] = byte_count;

		for (cont = 0; cont < byte_count; cont += 2) {
			req[7 + cont] = data[cont + 1];
			req[8 + cont] = data[cont];
		}

		crc = CRC16(req, cont + 7);
		req[cont + 7] = make8(crc, 0);
		req[cont + 8] = make8(crc, 1);
		break;
	default:
		break;
	}

	return 0;
}

uint8_t check_CRC(uint8_t *resp, modbus_command_t command) {
	uint8_t *arr;
	uint16_t ar_size, crc_check, crc_in, cont;

	switch (command) {
	case READ_HOLDING_REGISTERS_COMMAND:
	case READ_COILS_COMMAND:
	case READ_DISCRETE_INPUT_COMMAND:
		ar_size = resp[2] + 3;
		break;
	case WRITE_MULTIPLE_REGISTERS_COMMAND:
		ar_size = 6;
		break;
	default:
		break;
	}

	arr = NULL;
	arr = (uint8_t *) malloc((size_t) (ar_size * sizeof(uint8_t)));

	if (arr == NULL) {
		free(arr);
		return 0;
	}

	for (cont = 0; cont < ar_size; ++cont)
		arr[cont] = resp[cont];

	crc_in = make16(resp[ar_size + 1], resp[ar_size]);
	crc_check = CRC16(arr, ar_size);
	free(arr);

	return (uint8_t) (crc_check == crc_in);
}

uint8_t mount_modbus_response(modbus_response_t *response, uint8_t *resp) {
	int cont, size;

	size = resp[2];

	response->address = resp[0];
	response->function = resp[1];
	response->response_size = size;

	for (cont = 0; cont < size; ++cont)
		response->data[cont] = resp[cont + 3];

	response->crc = make16(resp[size + 4], resp[size + 3]);

	return 0;
}

uint8_t make_transaction(modbus_request_t *request, modbus_response_t *response) {
	uint8_t *req, *resp, resul;
	uint16_t resp_size, req_size;

	resul = 0;
	resp_size = 0;
	req_size = 0;

	switch (request->function) {
	case READ_DISCRETE_INPUT_COMMAND:
	case READ_COILS_COMMAND:
		if (request->size <= 8)
			resp_size = (size_t) (request->size / 8 + 5);
		else
			resp_size = (size_t) (request->size / 8 + 6);
		req_size = REQUEST_SIZE;
		break;
	case READ_HOLDING_REGISTERS_COMMAND:
		resp_size = (size_t) (request->size << 1) + 5;
		req_size = REQUEST_SIZE;
		break;
	case WRITE_MULTIPLE_REGISTERS_COMMAND:
		resp_size = (size_t) 6;
		req_size = 9 + request->byte_count;
		break;
	}

	req = NULL;
	req = (uint8_t *) malloc((size_t) (req_size * sizeof(uint8_t)));

	if (req == NULL) {
		free(req);
		return 1;
	}

	make_request(request->address, request->start_address, request->size,
			request->byte_count, request->data, request->function, req);

	resp = NULL;
	resp = (uint8_t *) malloc(resp_size * sizeof(uint8_t));

	if (resp == NULL) {
		free(req);
		free(resp);
		return 1;
	}

	resul = serial_transaction(req, resp, req_size, resp_size);
	free(req);

	if (resul != 0) {
		free(resp);
		return resul;
	}

	if (!check_CRC(resp, request->function)) {
		free(resp);
		return 2;
	}

	resul = mount_modbus_response(response, resp);
	free(resp);

	if (resul != 0)
		return resul;

	return 0;
}

uint8_t read_holding_registers(uint8_t dev_addr, uint16_t from, uint16_t size,
		uint16_t *to) {
	modbus_response_t *resp;
	modbus_request_t *req;

	req = NULL;
	req = (modbus_request_t *) malloc((size_t) (sizeof(modbus_request_t)));
	resp = NULL;
	resp = (modbus_response_t *) malloc((size_t) (sizeof(modbus_response_t)));

	if (req == NULL || resp == NULL) {
		free(req->data);
		free(req);
		free(resp->data);
		free(resp);
		return 1;
	}

	req->address = dev_addr;
	req->start_address = from;
	req->size = size;
	req->function = READ_HOLDING_REGISTERS_COMMAND;
	resp->data = (uint8_t *) to;

	if (make_transaction(req, resp) != 0) {
		free(req);
		free(resp);
		return 1;
	}

	free(req);
	free(resp);

	return 0;
}
