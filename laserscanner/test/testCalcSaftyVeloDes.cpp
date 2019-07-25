// import standard c library (atoi, atof, ...)
#include <cstdlib>
#include <cmath>

// import IO library (cout, endl, files,  ....)
#include <iostream>
#include <fstream>

#include <CalcSaftyVelocityDesired.hpp>
#include <LaserScanner.hpp>

using namespace std;

int main(int argc, char *argv[]) 
{
	std::cout << "LaserScanner started..." << std::endl;
	
	double dangerzone = 1.0; // 1[m]
	double min_per_distance = 0.05; // 0.05[m]
	double maxVelocity = 1.0; // [m/s]
	
	eeros::math::Matrix< 1, 2 > desiredVeloXY;
	desiredVeloXY(0,0) = 1.2; // [m/s]
	desiredVeloXY(0,1) = -0.52;
	
	CalcSaftyVelocityDesired calcSaftyVelo(desiredVeloXY, maxVelocity, dangerzone, min_per_distance);
	
	int start = 44;
	int end = 725;
	
	LaserScanner laserScanner(start,end);
	
	while(!laserScanner.openScanner("/dev/ttyS0")) {
		sleep(1);
	}

	std::ofstream fileIn;
	fileIn.open("/home/stefan/Projects/laserscanner/transit/FuerMatlab/scanDatenIn.txt");

	std::ofstream fileOut1;
	fileOut1.open("/home/stefan/Projects/laserscanner/transit/FuerMatlab/importantData.txt");
	
	std::ofstream fileOut2;
	fileOut2.open("/home/stefan/Projects/laserscanner/transit/FuerMatlab/newDesiredVelo.txt");
	
	if(laserScanner.scan()){

		std::cout << "scanned " << std::endl;
		
		eeros::math::Matrix<MAX_MATRIX_SIZE,2> distancAndAngle = laserScanner.getScannData();
	
		std::cout << "scannData : " << distancAndAngle(laserScanner.getDataSize()-1 ,0 )  << " angle: "<< distancAndAngle(laserScanner.getDataSize()-1,1)  << " DataSize: " << laserScanner.getDataSize()-1<<std::endl;
	
		for (int i = 0; i < laserScanner.getDataSize(); i++){
			fileIn << distancAndAngle(i ,0 );
			fileIn << '\t' << distancAndAngle(i ,1);
			fileIn << endl;
		}
	
		if(calcSaftyVelo.calcSaftyVeloDes(distancAndAngle)){
			
			eeros::math::Matrix<MAX_MATRIX_SIZE,4> importantData;
			importantData.zero();
			
			importantData = calcSaftyVelo.getImportantData();
			
			for (int i = 0; i < calcSaftyVelo.getSizeOfImpotrantData()+2; i++){
			fileOut1 << importantData(i,0);
			fileOut1 << '\t' << importantData(i,1);
			fileOut1 << '\t' << importantData(i,2);
			fileOut1 << '\t' << importantData(i,3);
			fileOut1 << endl;
			}
			
			eeros::math::Matrix< 1, 2 > saftyVelo = calcSaftyVelo.getSaftyVelocity();
			fileOut2 << saftyVelo(0,0);
			fileOut2 << '\t' << saftyVelo(0,1);
			
			
			
			std::cout << "saftyVelox: "<< saftyVelo(0,0) << " saftyVeloy: " <<saftyVelo(0,1) << std::endl;
		}
		else{
			
			std::cout << " error " << std::endl;
		}
	}
	else{
		std::cout << "not scanned " << std::endl;
	}
	
	fileIn.close();
	fileOut1.close();
	fileOut2.close();
	
	
	
	std::cout << "LaserScanner ended..." << std::endl;
	
	return 0;
}