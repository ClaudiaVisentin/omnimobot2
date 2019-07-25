#include <cstdlib>
#include <iostream>
#include <fstream>
// #include <math.h>
#include <cmath>





int main() {
	std::cout << "Start i-test" << std::endl;

	double test;
// 	test = sqrt(-1.0);
	test = -1/tan(0);
	
	
	if (std::isnan(test)) {
		std::cout << "nan: " <<test<< std::endl;
	}
	else {
		std::cout << "nicht gut!" << std::endl;
	}
	return -3;
	
}
