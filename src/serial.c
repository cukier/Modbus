#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>
#include "serial.h"

static int fd;
static char *porta;
static uint32_t baud;

#ifndef TIME_OUT
#define TIME_OUT	300
#endif

int serial_init(char p[], uint32_t b) {

	if (strlen(p) <= 0)
		return -1;

	if (b == 0)
		return -1;

	baud = b;
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

	if (baud != 0)
		serial_set_port(baud);
	else
		return -1;

	return 0;
}

int serial_close_port(void) {
	int r = -1;

	free(porta);
	r = close(fd);
	fd = -1;
	baud = 0;

	return r;
}

int serial_transaction(uint8_t *tx, uint8_t *rx, uint16_t msg_size,
		uint16_t resp_size) {
	int n, tries;

	if (serial_open_port() == -1) {
		serial_close_port();
		return -1;
	}

	tcflush(fd, TCIFLUSH);

	if (write(fd, tx, msg_size) == -1)
		return -1;

	n = -1;
	tries = TIME_OUT;

	while ((n == -1) && ((tries--) > 0)) {
		n = read(fd, rx, resp_size);
		usleep(1000);
	}

	serial_close_port();

	if (n == -1) {
		return -1;
	}

	return n;
}
