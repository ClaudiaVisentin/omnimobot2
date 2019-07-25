#include <unistd.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>

#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>

int main(int argc, char *argv[])
{
	
	
	std::cout << "Laser Scanner Test Application started..." << std::endl;

	int fd = ::open("/dev/ttyPSC1" , O_RDWR | O_NONBLOCK  | O_FSYNC); // init     "/dev/ttyPSC1"
//	int fd = ::open("/dev/ttyUSB0" , O_RDWR | O_NONBLOCK  | O_FSYNC); // init     "/dev/ttyPSC1"
//	int fd = ::open("/dev/ttyS0" , O_RDWR | O_NONBLOCK  | O_FSYNC); // init     "/dev/ttyPSC1"
	
	usleep(10000);
	
	/* set the other settings (in this case, 115200 8N1) */
	struct termios settings;
	tcgetattr(fd, &settings);
	

	speed_t baud = B115200; /* baud rate */
	cfsetospeed(&settings, baud); /* baud rate */
	cfsetispeed(&settings, baud); /* baud rate */
	settings.c_cflag &= ~PARENB; /* no parity */
	settings.c_cflag &= ~CSTOPB; /* 1 stop bit */
	settings.c_cflag &= ~CSIZE;
	settings.c_cflag |= CS8 | CLOCAL; /* 8 bits */
	settings.c_lflag &=(~ICANON & ~ECHO);
	settings.c_oflag &= ~OPOST; /* raw output */
	settings.c_iflag &= ~INPCK;
	
	//settings.c_iflag |= IGNBRK;
	//settings.c_iflag &= ~BRKINT;
	//settings.c_iflag |= ICRNL;
	
	//settings.c_cflag |= CREAD;

	tcsetattr(fd, TCSANOW, &settings); /* apply the settings */
	tcflush(fd, TCOFLUSH);
	usleep(10000);
	
	constexpr int N = 1024;
	char buffer[N + 1];
	
	strcpy(buffer, "VV\n");
	
	write(fd, buffer, strlen(buffer));
	//fsync(fd);
	usleep(10000);
	
	for (int i = 0; i < 10; i++) {
		int n = read(fd, buffer, N);
		if (n > 0 && n <= N) {
			
			buffer[n] = 0;
			std::cout << "read ok: " << buffer << std::endl;
		}
		else {
			std::cout << "cannot read" << std::endl;
		}
		usleep(15000);
	}
	return 0;

}