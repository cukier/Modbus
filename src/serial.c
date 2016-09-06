#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>
#include "serial.h"

#ifndef TRIES
#define TRIES	1000
#endif

static int fd;
static char *porta;
static uint32_t baud_rate;

int u8_strcpy(uint8_t *dest, const uint8_t *src, size_t u8size, size_t offset) {
	size_t cont;

	for (cont = 0; cont < u8size; ++cont) {
		dest[cont + offset] = src[cont];
	}

	return 0;
}

int serial_init(char p[], uint32_t b) {

	if (strlen(p) <= 0)
		return -1;

	if (b == 0)
		return -1;

	baud_rate = b;
	fd = -1;
	porta = NULL;
	porta = (char *) malloc(strlen(p) * sizeof(char));

	if (porta == NULL)
		return -1;

	strcpy(porta, p);

	return 0;
}

int serial_set_port(uint32_t baud_rate) {
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

int serial_open_port(void) {

	if (strlen(porta) > 4) {
		fd = open(porta, O_RDWR | O_NOCTTY | O_NDELAY);

		if (fd == -1)
			return -1;
	} else
		return -1;

	if (baud_rate != 0)
		serial_set_port(baud_rate);
	else
		return -1;

	return 0;
}

int serial_close_port(void) {
	int r = -1;

	r = close(fd);
	fd = -1;

	return r;
}

int serial_close(void) {

	free(porta);
	baud_rate = 0;
	return serial_close_port();

}

size_t serial_transaction(uint8_t *tx, uint8_t *rx, uint16_t msg_size,
		uint16_t resp_size) {
	int n, m_tries;
	size_t m_size;
	uint8_t *ptr;

	if (serial_open_port() == -1) {
		serial_close_port();
		return -1;
	}

	n = -1;
	m_tries = TRIES;

	tcflush(fd, TCIFLUSH);
	if (write(fd, tx, msg_size) == -1)
		return -1;

#ifdef TIME_FRAME
	usleep(TIME_FRAME);
#endif
	ptr = NULL;
	m_size = 0;
	while ((m_size != resp_size) && ((m_tries--) > 0)) {
		usleep(100);
		n = read(fd, rx, resp_size);
		if (n != -1 && n > 0) {
			m_size += n;
			ptr = (uint8_t *) realloc(ptr, m_size * sizeof(uint8_t));

			if (ptr == NULL)
				break;

			u8_strcpy(ptr, rx, n, (m_size - n));
		}
	}

	serial_close_port();

	if (m_size > 0)
		u8_strcpy(rx, ptr, m_size, 0);

	free(ptr);

	if (n == -1) {
		return -1;
	}

	return m_size;
}
