#include <unistd.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>


int main(int argc, char *argv[])
{
	
	
	std::cout << "Laser Scanner Test Application started..." << std::endl;

	for(int i = 0; i <=1; i++){
	
	std::fstream communication("/dev/ttyS0", std::ios::out | std::ios::in | std::ios::binary); // init
	
	
	std::string s;
	
	
		communication << "MD0384038400001" << std::endl; // alle in s schreiben, damit alles abgefangen wird... s hat immer nur die letzten daten
		communication >> s;
		std::cout << s << std::endl;
		communication >> s;
		std::cout << s << std::endl;
		communication >> s;
		std::cout << s << std::endl;
		communication >> s;
		std::cout << s << std::endl;
		communication >> s;
		std::cout << s << std::endl;
		communication >> s; // das letzte s
		std::cout << s << std::endl;
		communication.clear();
		
		unsigned char c0 = s[0] - '0'; // im ASCII wird die 30H abgezogen
		unsigned char c1 = s[1] - '0';
		unsigned char c2 = s[2] - '0';
		
		
//		std::cout << "distance: " <<(int)c0 << "    " << (int)c1 << "   " << (int)c2 << std::endl;
		
		int distance = ((c0 & 0x3f) << 12) | ((c1 & 0x3f) << 6) | (c2 & 0x3f); // c0 und 00111111 gibt richtiges Bitmuster und schiebt es 12 plätze rüber 111100000000000000 |(oder) werden oder verknüpft mit dem nächsten Bitmuster
		std::cout << "distance: " << distance << std::endl;
	}



	return 0;
}
