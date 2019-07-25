#include <CommandURG04LX.hpp>

#include <array>
#include <iostream>


int main(int argc, char *argv[]) 
{

	std::cout << "CommandURG04LX test started..." << std::endl;
	
	ScanCommand command;
	command.setStartValue(46);
	command.setEndValue(600);
	
	printf(" Command = %s",command.toCString());
	
	std::cout << "Command: " << command;
	
	char buffer[17] = "MD0046060000001\n";
	
	bool check =command.check(buffer);
	
	if(check){
		printf("Check is true: %d \n",check);
	}
	else{
		printf("Check is false: %d \n",check);
	}
		

// 	int a[8];
// 	int i;
// 	for (i = 0; i< 8; i++){
// 		a[i] = i;
// 		printf("%i \n",a[i]);
// 	}
	
	std::cout << "CommandURG04LX test ended..." << std::endl;
	
	return 0;
}