#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>
#include "serial.h"
#include "modbus.h"

int open_port(char *porta) {
	int fd;

	fd = open(porta, O_RDWR | O_NOCTTY | O_NDELAY);

	return fd;
}

int set_port(int baud_rate, int fd) {
	int ret;
	struct termios options;
	speed_t speed;

	switch (baud_rate) {
	case 300:
		speed = B300;
		break;
	default:
	case 9600:
		speed = B9600;
		break;
	case 19200:
		speed = B19200;
		break;
	case 57600:
		speed = B57600;
		break;
	case 115200:
		speed = B115200;
		break;
	}

	ret = tcgetattr(fd, &options);

	if (ret != 0)
		return ret;

	ret = cfsetispeed(&options, speed);

	if (ret != 0)
		return ret;

	ret = cfsetospeed(&options, speed);

	if (ret != 0)
		return ret;

//	options.c_cflag |= (CLOCAL | CREAD);
//	options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
//	options.c_cflag |= CS8;
//	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
//	options.c_oflag &= ~OPOST;
//	options.c_iflag &= ~(IXON | IXOFF | IXANY);

	options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR
			| ICRNL | IXON);
	options.c_oflag &= ~OPOST;
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_cflag &= ~(CSIZE | PARENB);
	options.c_cflag |= CS8;

	ret = tcsetattr(fd, TCSANOW, &options);

	if (ret != 0)
		return ret;

	return 0;
}

int make_transaction(int fd, uint8_t *msg, uint8_t *resp, int msg_size,
		int resp_size) {
	int n = -1, tries = 1000;

	tcflush(fd, TCIFLUSH);
	write(fd, msg, msg_size);

	while (n == -1 && (tries--) > 0) {
		n = read(fd, resp, resp_size);
		usleep(500000);
	}

	if (n == -1)
		return -1;

	return n;
}

bool check_response(uint8_t *response, uint8_t *request) {
	bool check = true;
	int cont;

	for (cont = 0; cont < 8; ++cont) {
		if (cont == 5)
			check &= response[cont] == 0x00;
		else
			check &= response[cont] == request[cont];
	}

	return check;
}

int clear_response(uint8_t *response, int size) {
	int cont;

	for (cont = 0; cont < size; ++cont)
		response[cont] = 0;

	return 0;
}

//int make_read_transaction(int fd, int dev_addr, uint8_t *response, int from,
//		int count) {
//	int n = -1;
//	uint8_t *request;
//
//	request = (uint8_t *) malloc(REQUEST_SIZE);
//
//	clear_response(response, count);
//	make_read_request(dev_addr, from, count);
//	n = make_transaction(fd, request, response, count, count, NULL);
//
//	free(request);
//
//	if (n != count) {
//		return -1;
//	} else if (!check_response(response, request))
//		return -2;
//
//	return n;
//}
