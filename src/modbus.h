/*
 * modbus.h
 *
 *  Created on: 03/12/2014
 *      Author: cuki
 */

#ifndef MODBUS_H_
#define MODBUS_H_

#include <stdint.h>

#define READ_COILS					0x01
#define READ_DISCRETE_INPUTS		0x02
#define READ_HOLDING_REGISTERS		0x03
#define READ_INPUT_REGISTERS		0x04
#define WRITE_SINGLE_COIL			0x05
#define WRITE_SINGLE_REGISTER		0x06
#define WRITE_MULTIPLE_COILS		0x0F
#define WRITE_MULTIPLE_REGISTERS	0x10

#define HIGH						1
#define LOW							0
#define REQUEST_SIZE				8
#define BUFFER_S					256

struct resp_str {
	uint8_t addr;
	uint8_t cmd;
	uint8_t resp_size;
	uint8_t *resp;
	uint16_t crc;
};

struct req_str {
	uint8_t addr;
	uint8_t cmd;
	uint16_t from;
	uint16_t to;
	uint16_t crc;
};

typedef union req_un {
	struct req_str structure;
	uint8_t string[sizeof(struct req_str)];
} req_un_t;

union {
	float a;
	uint8_t b[sizeof(float)];
} un;

extern const uint16_t wCRCTable[];
extern uint8_t make_byte(uint16_t word, uint8_t index);
extern uint16_t make_word(uint8_t hiByte, uint8_t loByte);
extern uint16_t swap16(uint16_t in);
extern uint16_t CRC16(uint8_t *nData, uint16_t wLength);
extern uint8_t *make_request(uint8_t dev_addr, uint16_t from, uint16_t to,
		uint8_t type);
extern uint8_t *make_read_request(uint8_t dev_addr, uint16_t from, uint16_t nr);
extern uint8_t *make_write_request(uint8_t dev_addr, uint16_t reg_addr,
		uint16_t reg_value);
extern uint8_t *make_discrete_inputs_request(uint8_t dev_addr,
		uint16_t reg_addr, uint16_t reg_value);
extern uint8_t *make_read_coils_request(uint8_t dev_addr, uint16_t reg_addr,
		uint16_t reg_value);
extern uint8_t *alloc_response(uint8_t *request, int *size);
//extern int get_device(int fd, uint8_t dev_addr, uint16_t reg_addr,
//		uint16_t reg_data, uint8_t *device_memorie);
extern uint8_t *fromFloat(float in);
extern float toFloat(uint8_t *in);

#endif /* MODBUS_H_ */
