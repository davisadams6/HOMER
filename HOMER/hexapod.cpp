#include <iostream>
//#include <string.h>
//#include <bitset>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <math.h>

#define HEXAPOD_BAUDRATE B57600
#define HEXAPOD_PORT "/dev/ttyUSB0"

int hexapod_fd;

int main(){

	init_hexapod();

	return 0;

}

int init_hexapod(){

	struct termios options;
	int ret = 0;

	hexapod_fd = open(HEXAPOD_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
	if (hexapod_fd<0) {
		std::cerr << "The serial port did not open correctly!" << std::endl;
		return EXIT_FAILURE;
	}

	tcgetattr(hexapod_fd,&options);
	options.c_cflag = HEXAPOD_BAUDRATE | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 1;
	tcflush(hexapod_fd,TCIOFLUSH);
	tcsetattr(hexapod_fd,TCSANOW,&options);


	return 0;
}
