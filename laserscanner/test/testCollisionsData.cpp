// import standard c library (atoi, atof, ...)
#include <cstdlib>
#include <cmath>

// import IO library (cout, endl, files,  ....)
#include <iostream>
#include <fstream>

#include <LaserScanner.hpp>
#include <CollisionDetection.hpp>

using namespace std;


int main(int argc, char *argv[]) 
{
	std::cout << "LaserScanner started..." << std::endl;

	
	CollisionDetection calcImportantData;

	int start = 44; // in constants def
	int end = 725;
	
	LaserScanner laserScanner(start,end);

	while(!laserScanner.openScanner("/dev/ttyPSC1")) { // "/dev/ttyS0"
		sleep(1);
	}
	
	std::ofstream fileCollIn;
	fileCollIn.open("ScanDatenIn.txt");

	std::ofstream fileOut;
	fileOut.open("CollData.txt");
	
	if(laserScanner.scan()){
		
		int dataSize =  laserScanner.getOutDataSize();

		std::cout << "scanned " << std::endl;
		DataMatrix distancAndAngle = laserScanner.getScannData();
		
		std::cout << "scannData : " << distancAndAngle(laserScanner.getOutDataSize()-1 ,0)  << " angle: "<< distancAndAngle(laserScanner.getOutDataSize()-1,1)  << " DataSize: " << laserScanner.getOutDataSize()-1<<std::endl;
		
		
		for (int i = 0; i < dataSize; i++){
			fileCollIn << distancAndAngle(i ,0 );
			fileCollIn << '\t' << distancAndAngle(i ,1);
			fileCollIn << endl;
		}
		
		if(calcImportantData.calcCollisionsData(distancAndAngle, dataSize)){
			
			CollisionDataMatrix importantData;
			importantData.zero();
			
			importantData = calcImportantData.getCollisionsData();
			
			std::cout << "SizeImportantData: " << calcImportantData.getSizeCollisionsData()  << " max: "<< importantData(calcImportantData.getSizeCollisionsData()-1,3)  << " importantdata(0,2): "<< importantData(0,2) << " importantdata(0,3): "<< importantData(0,3)  << std::endl;

			for (int i = 0; i < calcImportantData.getSizeCollisionsData()+2; i++){
			fileOut << importantData(i,0);
			fileOut << '\t' << importantData(i,1);
			fileOut << '\t' << importantData(i,2);
			fileOut << '\t' << importantData(i,3);
			fileOut << endl;
			}
			
		}
		
	}
	else{
		std::cout << "not scanned " << std::endl;
	}
	
	fileCollIn.close();
	fileOut.close();
	
	std::cout << "LaserScanner ended..." << std::endl;
	
	return 0;
}