// import standard c library (atoi, atof, ...)
#include <cstdlib>
#include <cmath>

// import IO library (cout, endl, files,  ....)
#include <iostream>
#include <fstream>

#include <LaserScanner.hpp>

using namespace std;


int main(int argc, char *argv[]) 
{
	std::cout << "LaserScanner started..." << std::endl;
	
	LaserScanner laserScanner;
	
	while(!laserScanner.openScanner("/dev/ttyS0")) {
		sleep(1);
	}
	
	int start = 44;
	int end = 725;
	
	std::ofstream file;
	file.open("/home/stefan/Projects/laserscanner/transit/FuerMatlab/scanDaten.txt");

	
	if(laserScanner.scan()){

		std::cout << "scanned " << std::endl;
		DataMatrix distancAndAngle = laserScanner.getScannData();
		
		std::cout << "scannData : " << distancAndAngle(laserScanner.getOutDataSize() ,0 )  << " angle: "<< distancAndAngle(laserScanner.getOutDataSize(),1)  << " DataSize: " << laserScanner.getOutDataSize()<<std::endl;
		
		for (int i = laserScanner.getOutDataSize(); i >= 0; i--){
			file << distancAndAngle(i ,0 );
			file << '\t' << distancAndAngle(i ,1);
			file << endl;
		}
		
	}
	else{
		std::cout << "not scanned " << std::endl;
	}
	
	file.close();
	
	std::cout << "LaserScanner ended..." << std::endl;
	
	return 0;
}