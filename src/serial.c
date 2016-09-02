#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>
#include "serial.h"

int serial_open_port(char *porta) {
	int fd;

	fd = open(porta, O_RDWR | O_NOCTTY | O_NDELAY);

	return fd;
}

int serial_close_port(int fd) {

	close(fd);

	return 0;
}

int serial_set_port(int baud_rate, int fd) {
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

int serial_transaction(int fd, uint8_t *tx, uint8_t *rx, int msg_size,
		int resp_size) {
	int n = -1, tries = 1000;

	tcflush(fd, TCIFLUSH);
	write(fd, tx, msg_size);

	while (n == -1 && (tries--) > 0) {
		n = read(fd, rx, resp_size);
		usleep(500000);
	}

	if (n == -1)
		return -1;

	return n;
}
