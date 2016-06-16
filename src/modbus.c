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

long swap_w(long word) {
	long aux;

	aux = (word & 0xFF00) >> 8;
	aux |= (word & 0x00FF) << 8;

	return aux;
}

#ifdef USE_UART1
#use rs232(uart1, baud=UART1_BAUD_RATE, stream=sl1)
#INT_RDA
void isr_rda() {
	clear_interrupt(INT_RDA);
	uc_set_timer_rda();
	uc_setup_timer_rda();
	buffer_rda[index_rda++] = fgetc(sl1);
	if (index_rda >= RDA_BUFFER_SIZE)
	index_rda = 0;
}
#if (SL1_TYPE == SL_MASTER_SLAVE)
#INT_TIMER0
void isr_rda_timer() {
	clear_interrupt(RDA_INTERRUPT);
	index_rda = 0;
	uc_set_timer_rda();
	uc_turn_off_timer_rda();
	slave_respond = TRUE;
}
#endif
#endif

#ifdef USE_UART2
#use rs232(uart2, baud=UART2_BAUD_RATE, stream=sl2)
#INT_RDA2
void isr_rda2() {
	clear_interrupt(INT_RDA2);
	uc_set_timer_rda2();
	uc_setup_timer_rda2();
	buffer_rda2[index_rda2++] = fgetc(sl2);
	if (index_rda2 >= RDA2_BUFFER_SIZE)
	index_rda2 = 0;
}
#if (SL2_TYPE == SL_MASTER_SLAVE)
#INT_TIMER1
void isr_rda2_timer() {
	clear_interrupt(RDA2_INTERRUPT);
	uc_set_timer_rda2();
	uc_turn_off_timer_rda2();
	slave_respond = TRUE;
}
#endif
#endif

modbus_exception_t slave_response(door_t door) {
	long cont, crc, register_value, register_address, b_count;
	int *resp, *buff;
	size_t size;
	modbus_fields_t field;

	size = 0;
	buff = (door == SL1_DOOR) ? buffer_rda : buffer_rda2;
	register_value = make16(buff[REGISTER_VALUE_H], buff[REGISTER_VALUE_L]);
	register_address = make16(buff[REGISTER_ADDRESS_H],
			buff[REGISTER_ADDRESS_L]);
	b_count = buff[BYTE_COUNT];

	if (my_address == buff[ADDRESS]) {
		switch (buff[FUNCTION]) {
		case READ_HOLDING_REGISTERS_COMMAND:

			if (register_value == 0 || register_value > 0x007D) {
				size = 5;
				resp = NULL;
				resp = (int *) malloc(size);

				if (resp == NULL) {
					free(resp);
					return OUT_OF_MEMORY_EXCEPTION;
				}

				resp[0] = my_address;
				resp[1] = 0x83;
				resp[2] = 0x03;
				crc = CRC16(resp, 3);
			} else if (register_value + register_address >= UINT16_MAX) {
				size = 5;
				resp = NULL;
				resp = (int *) malloc(size);

				if (resp == NULL) {
					free(resp);
					return OUT_OF_MEMORY_EXCEPTION;
				}

				resp[0] = my_address;
				resp[1] = 0x83;
				resp[2] = 0x02;
				crc = CRC16(resp, 3);
			} else {
				size = (size_t)((register_value * 2) + 5);
				resp = NULL;
				resp = (int *) malloc(size);

				if (resp == NULL) {
					free(resp);
					return OUT_OF_MEMORY_EXCEPTION;
				}

				resp[0] = my_address;
				resp[1] = READ_HOLDING_REGISTERS_COMMAND;
				resp[2] = (int) register_value * 2;
				for (cont = 0; cont < register_value; ++cont) {
					resp[(2 * cont) + 3] = read_eeprom(
							register_address + (2 * cont));
					resp[(2 * cont) + 4] = read_eeprom(
							register_address + ((2 * cont) + 1));
				}
				crc = CRC16(resp, (register_value * 2) + 3);
			}

			resp[size - 2] = make8(crc, 1);
			resp[size - 1] = make8(crc, 0);
			break;

		case WRITE_SINGLE_REGISTER_COMMAND:

			size = (size_t)(8);
			resp = NULL;
			resp = (int *) malloc(size);

			if (resp == NULL) {
				free(resp);
				return OUT_OF_MEMORY_EXCEPTION;
			}

			write_eeprom(register_address * 2, make8(register_value, 1));
			write_eeprom((register_address * 2) + 1, make8(register_value, 0));

			resp[0] = my_address;
			resp[1] = WRITE_SINGLE_REGISTER_COMMAND;
			resp[2] = (int) make8(register_address, 1);
			resp[3] = (int) make8(register_address, 0);
			resp[4] = (int) make8(register_value, 1);
			resp[5] = (int) make8(register_value, 0);
			crc = CRC16(resp, 6);
			resp[6] = (int) make8(crc, 1);
			resp[7] = (int) make8(crc, 0);

			break;

		case WRITE_MULTIPLE_REGISTERS_COMMAND:

			size = (size_t)(8);
			resp = NULL;
			resp = (int *) malloc(size);

			if (resp == NULL) {
				free(resp);
				return OUT_OF_MEMORY_EXCEPTION;
			}

			for (cont = 0; cont < b_count; ++cont) {
				write_eeprom((2 * register_address + cont),
						buff[cont + BYTE_COUNT + 1]);
			}

			resp[0] = my_address;
			resp[1] = WRITE_MULTIPLE_REGISTERS_COMMAND;
			resp[2] = (int) make8(register_address, 1);
			resp[3] = (int) make8(register_address, 0);
			resp[4] = (int) make8(register_value, 1);
			resp[5] = (int) make8(register_value, 0);
			crc = CRC16(resp, 6);
			resp[6] = (int) make8(crc, 1);
			resp[7] = (int) make8(crc, 0);

			break;

		default:
			break;
		}
	}

	for (cont = 0; cont < size; ++cont) {
#ifdef USE_UART1
#if (SL1_TYPE == SL_MASTER_SLAVE)
		fputc(resp[cont], sl1);
#endif
#endif

#ifdef USE_UART2
#if (SL2_TYPE == SL_MASTER_SLAVE)
		fputc(resp[cont], sl2);
#endif
#endif
	}

	free(resp);

	if (door == SL1_DOOR)
		flush_buffer(buffer_rda, &index_rda);
	else
		flush_buffer(buffer_rda2, &index_rda2);

	return NO_EXCEPTION;
}

int modbus_init(int id, int slave_address) {

	my_address = id;
	slv_address = slave_address;

#ifdef USE_UART1
	uc_turn_off_timer_rda();
	uc_clear_rda_interrupt();
	uc_set_timer_rda();
	clear_interrupt(INT_RDA);
	enable_interrupts(INT_RDA);
	clear_interrupt(RDA_INTERRUPT);
	enable_interrupts(RDA_INTERRUPT);
#endif

#ifdef USE_UART2
	uc_clear_rda2_interrupt();
	uc_turn_off_timer_rda2();
	uc_set_timer_rda2();
	clear_interrupt(INT_RDA2);
	enable_interrupts(INT_RDA2);
	clear_interrupt(RDA2_INTERRUPT);
	enable_interrupts(RDA2_INTERRUPT);
#endif

	enable_interrupts(GLOBAL);

	return 0;
}

int parse_error(modbus_exception_t error, char *msg) {
	switch (error) {
	case NO_EXCEPTION:
		strcpy(msg, "no errors");
		break;
	case TIMEOUT_EXCEPTION:
		strcpy(msg, "time out");
		break;
	case OUT_OF_MEMORY_EXCEPTION:
		strcpy(msg, "out of memory");
		break;
	case CRC_EXCEPTION:
		strcpy(msg, "crc error");
		break;
	case WRONG_RESPONSE_EXCEPTION:
		strcpy(msg, "wrong response");
		break;
	case MODBUS_EXCEPTION:
		strcpy(msg, "modbus exception");
		break;
	default:
		strcpy(msg, "unknown error");
		break;
	}
	return 0;
}

long CRC16(int *nData, long wLength) {
	int nTemp;
	long wCRCWord = 0xFFFF;

	while (wLength--) {
		nTemp = *nData++ ^ wCRCWord;
		wCRCWord >>= 8;
		wCRCWord ^= wCRCTable[nTemp];
	}

	return wCRCWord;
}

int make_request(int dev_addr, long from, long size, modbus_command_t command,
		int *req) {
	long crc;

	req[0] = dev_addr;
	req[1] = (int) command;
	req[2] = make8(from, 1);
	req[3] = make8(from, 0);
	req[4] = make8(size, 1);
	req[5] = make8(size, 0);
	crc = CRC16(req, 6);
	req[6] = make8(crc, 0);
	req[7] = make8(crc, 1);

	return 0;
}

void flush_buffer(int *buffer, long *index) {

	for (erase_cont = 0; erase_cont < *index; ++erase_cont)
		buffer[erase_cont] = 0;

	*index = 0;
}

//funcao que leia a resposta do plc
modbus_exception_t read_plc_response(int *req, int *resp, door_t sl) {
	long i, time_out, expected_size, cont;
	modbus_command_t cmd;

	i = 0;
	time_out = 0;
	cmd = (modbus_command_t) req[1];
	expected_size = 0;

	if (sl == SL1_DOOR) {
#ifdef USE_UART1
		disable_interrupts(RDA_INTERRUPT);
		index_rda = 0;
#endif
	} else {
#ifdef USE_UART2
		disable_interrupts(RDA2_INTERRUPT);
		index_rda2 = 0;
#endif
	}

	switch (cmd) {
	case READ_HOLDING_REGISTERS_COMMAND:
		expected_size = (make16(req[4], req[5]) << 1) + 5;
		break;
	case READ_COILS_COMMAND:
	case READ_DISCRETE_INPUT_COMMAND:
		expected_size = make16(req[4], req[5]);
		if (expected_size <= 8)
			expected_size = (expected_size / 8) + 5;
		else
			expected_size = (expected_size / 8) + 6;
		break;
	}

	if (sl == SL1_DOOR) {
#ifdef USE_UART1
		for (i = 0; i < REQUEST_SIZE; ++i) {
			fputc(req[i], sl1);
		}

		while (!interrupt_active(RDA_INTERRUPT)) {
			delay_us(10);
			time_out++;
			if (time_out == 0xFFFF) {
				flush_buffer(buffer_rda, &index_rda);
				enable_interrupts(RDA_INTERRUPT);
				return TIMEOUT_EXCEPTION;
			}
		}

		uc_clear_rda_interrupt();
		uc_turn_off_timer_rda();
		uc_set_timer_rda();

		if (index_rda != 0 && index_rda == expected_size) {
			for (cont = 0; cont < index_rda; ++cont) {
				resp[cont] = buffer_rda[cont];
				buffer_rda[cont] = 0;
			}
			index_rda = 0;
		} else if (index_rda != expected_size) {
			flush_buffer(buffer_rda, &index_rda);
			enable_interrupts(RDA_INTERRUPT);
			return WRONG_RESPONSE_EXCEPTION;
		}
		enable_interrupts(RDA_INTERRUPT);
#endif
	} else {
#ifdef USE_UART2
		for (i = 0; i < REQUEST_SIZE; ++i) {
			fputc(req[i], sl2);
		}

		while (!interrupt_active(RDA2_INTERRUPT)) {
			delay_us(10);
			time_out++;
			if (time_out == 0xFFFF) {
				flush_buffer(buffer_rda2, &index_rda2);
				enable_interrupts(RDA2_INTERRUPT);
				return TIMEOUT_EXCEPTION;
			}
		}

		uc_clear_rda2_interrupt();
		uc_turn_off_timer_rda2();
		uc_set_timer_rda2();

		if (index_rda2 != 0 && index_rda2 == expected_size) {
			for (cont = 0; cont < index_rda2; ++cont) {
				resp[cont] = buffer_rda2[cont];
				buffer_rda2[cont] = 0;
			}
			index_rda2 = 0;
		} else if (index_rda2 != expected_size) {
			flush_buffer(buffer_rda2, &index_rda2);
			enable_interrupts(RDA2_INTERRUPT);
			return WRONG_RESPONSE_EXCEPTION;
		}
		enable_interrupts(RDA2_INTERRUPT);
#endif
	}

	return NO_EXCEPTION;
}
modbus_exception_t mount_modbus_response(modbus_rx_t *modbus_response,
		int *resp) {
	int cont, size, *data;

	data = NULL;
	size = resp[2];

	modbus_response->address = resp[0];
	modbus_response->function = resp[1];
	modbus_response->response_size = size;

	free(modbus_response->data);
	data = (int *) malloc((size_t) size);

	if (data == NULL) {
		free(data);
		return OUT_OF_MEMORY_EXCEPTION;
	}

	for (cont = 0; cont < size; ++cont)
		data[cont] = resp[cont + 3];

	modbus_response->data = data;
	modbus_response->crc = make16(resp[size + 4], resp[size + 3]);

	return NO_EXCEPTION;
}

short check_CRC(int *resp) {
	int *aux;
	long size, crc_check, crc_in, cont;

	size = resp[2] + 3;
	aux = (int *) malloc((size_t) size);

	if (aux == NULL) {
		free(aux);
		return FALSE;
	}

	for (cont = 0; cont < size; ++cont)
		aux[cont] = resp[cont];

	crc_in = make16(resp[size + 1], resp[size]);
	crc_check = CRC16(aux, size);
	free(aux);

	return crc_check == crc_in;;
}

modbus_exception_t make_transaction(int dev_addr, long from, long size,
		modbus_command_t command, modbus_rx_t *modbus_response, door_t sl) {
	int *req, *resp;
	size_t resp_size;
	modbus_exception_t resul;

	resul = NO_EXCEPTION;
	resp_size = 0;
	req = NULL;
	req = (int *) malloc((size_t) REQUEST_SIZE);

	if (req == NULL) {
		free(req);
		return OUT_OF_MEMORY_EXCEPTION;
	}

	make_request(dev_addr, from, size, command, req);

	switch (command) {
	case READ_DISCRETE_INPUT_COMMAND:
	case READ_COILS_COMMAND:
		if (size <= 8)
			resp_size = (size_t)(size / 8 + 5);
		else
			resp_size = (size_t)(size / 8 + 6);
		break;
	case READ_HOLDING_REGISTERS_COMMAND:
		resp_size = (size_t)(size << 1) + 5;
		break;
	}

	resp = NULL;
	resp = (int *) malloc(resp_size);

	if (resp == NULL) {
		free(req);
		free(resp);
		return OUT_OF_MEMORY_EXCEPTION;
	}

	resul = read_plc_response(req, resp, sl);
	free(req);

	if (resul != NO_EXCEPTION) {
		free(resp);
		return resul;
	}

	if (!check_CRC(resp))
		return CRC_EXCEPTION;

	resul = mount_modbus_response(modbus_response, resp);
	free(resp);

	if (resul != NO_EXCEPTION)
		return resul;

	return NO_EXCEPTION;
}

int get_word_mem(modbus_rx_t *device, long *resp) {
	long cont, size_bytes, size_array;
	int aux_h, aux_l;

	size_bytes = device->response_size;
	size_array = device->response_size / sizeof(long);

	for (cont = 0; cont < size_array; ++cont) {
		aux_h = device->data[2 * cont];
		aux_l = device->data[(2 * cont) + 1];
		resp[cont] = make16(aux_h, aux_l);
	}

	return 0;
}

int get_byte_mem(modbus_rx_t *device, int *resp) {
	int cont;

	for (cont = 0; cont < device->response_size; ++cont)
		resp[cont] = device->data[cont];

	return 0;
}

modbus_exception_t transport(int dev_addr, long from, long size, long *resp,
		modbus_command_t command, door_t sl) {
	modbus_rx_t *modbus_response;
	modbus_exception_t exception;

	modbus_response = NULL;
	exception = NO_EXCEPTION;
	modbus_response = (modbus_rx_t *) malloc(sizeof(modbus_rx_t));

	if (modbus_response == NULL) {
		free(modbus_response->data);
		free(modbus_response);
		return OUT_OF_MEMORY_EXCEPTION;
	}

	exception = make_transaction(dev_addr, from, size, command, modbus_response,
			sl);

	if (exception != NO_EXCEPTION) {
		free(modbus_response->data);
		free(modbus_response);
		return exception;
	}

	switch (command) {
	case READ_COILS_COMMAND:
	case READ_DISCRETE_INPUT_COMMAND:
		get_byte_mem(modbus_response, resp);
		break;
	case READ_HOLDING_REGISTERS_COMMAND:
		get_word_mem(modbus_response, resp);
		break;
	}

	free(modbus_response->data);
	free(modbus_response);

	return NO_EXCEPTION;

}

modbus_exception_t read_holding_registers(int dev_addr, long from, long size,
		long *to, door_t sl) {
	return transport(dev_addr, from, size, to, READ_HOLDING_REGISTERS_COMMAND,
			sl);
}

modbus_exception_t read_discrete_inputs(int dev_addr, long from, long size,
		long *to, door_t sl) {

	return transport(dev_addr, from, size, to, READ_DISCRETE_INPUT_COMMAND, sl);
}

modbus_exception_t read_coils(int dev_addr, long from, long size, long *to,
		door_t sl) {

	return transport(dev_addr, from, size, to, READ_COILS_COMMAND, sl);
}
