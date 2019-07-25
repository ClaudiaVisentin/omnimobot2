#include <iostream>
#include <unistd.h>
#include <fcntl.h>

#include "CommandURG04LX.hpp"

int main(int argc, char *argv[])
{
        const int buffer_size = 4096;
        char buffer[buffer_size];
        int fd, n;
		
		CommandURG04LX<16> c1("MD0044072500001\n");
		
		

        std::cout << "test-serial-c start" << std::endl;

        fd = open("/dev/ttyS0", O_RDWR | O_NONBLOCK | O_FSYNC);
// 		fd = open("/dev/ttyS0", O_RDWR | O_FSYNC);

        if (fd < 0)                                                                                                                                                          
        {                                                                                                                                                                    
                std::cout << "cannot open device" << std::endl;
                return 1;                                                                                                                                                    
        }                                                                                                                                                                    

        for (int i = 0; i< 2; i++)
		{
			std::cout << "clearing buffer..." << std::endl;
			while (read(fd, buffer, buffer_size) > 0);
			std::cout << "sending..." << std::endl;
			write(fd, "MD0044072500001\n", 16);   
// 			fd << c1;
			sleep(1);              
			std::cout << "receiving..." << std::endl;
			do {
				n = read(fd, buffer, buffer_size);
				if (n < 0) usleep(1000);
			} while (n < 0);
			if (n > 0)                                                                                                                                                           
			{                                                                                                                                                                    
					buffer[n] = 0;                                                                                                                                               
					std::cout << buffer << std::endl; 
			}
		}

        close(fd);

        std::cout << "test-serial-c stop" << std::endl;
        return 0;
}