#include <LaserScanner.hpp>
#include <iostream>
#include "../../omnimobot/include/omnimobot/constants.hpp"

using namespace omnimobot;

int main(int argc, char *argv[]) 
{
	try {
	std::cout << "LaserScanner test started..." << std::endl;
	
	int start = 369;
	int end = 399;
	
	LaserScanner laserScanner(start, end);
	
	while(!laserScanner.openScanner("/dev/ttyPSC1" )) {		// "/dev/ttyS0"   "/dev/ttyPSC1"
		sleep(1);
	};

	double cosAngle[omnimobot::angleSize];
	double sinAngle[omnimobot::angleSize];
	double tanAngle[omnimobot::angleSize];
	double tmpAngle = 0.0;
	
	for(int i = 0, step = 44; i < angleSize; i++, step++){		// step = 44 is the start step of the laserscanner
		
		if(step >= frontStep){
			tmpAngle = (step - frontStep) * angleResulution;
		}
		else{
			tmpAngle = -(frontStep - step )*angleResulution;
		}
		
		cosAngle[i] = cos(tmpAngle);
		sinAngle[i] = sin(tmpAngle);
		tanAngle[i] = tan(tmpAngle);
	}
	
	for (int i = 0; i< 5; i++){
		if(laserScanner.scan()){
			std::cout << "scanned " << std::endl;

			DataVector angle;
			DataVector distance;
			DataMatrix detectedPoints;	
			IndexMatrix criticalPoints;		
			
			DataMatrix distancAndAngle = laserScanner.getScannData();
			int sizeOfdataMatrix = laserScanner.getOutDataSize();
			const int startIndexCosSinTan = start - 44;	
			
			for (int i_in = 0, i_out = 0, u = startIndexCosSinTan; i_out < sizeOfdataMatrix; i_out++,u++){
				
				
				// winkel stimmen nicht
				angle(i_out) = distancAndAngle(i_out,1);
				distance(i_out) = distancAndAngle(i_out,0);
				detectedPoints(i_out,0) = cosAngle[u] * distance(i_out) + transStoSapostrophe; // tranlstion S to S apostrophe [m]
				detectedPoints(i_out,1) = sinAngle[u] * distance(i_out); // [m]
				
				if (distance(i_out) <= dangerzone && distance(i_out) >= minPermissibleDistance){
					criticalPoints(i_in,0) = i_out;
					criticalPoints(i_in,1) = u;
					i_in++;
				}
			}
			
			
			
			
			
			
			std::cout << "DataSize : " << laserScanner.getOutDataSize()   << std::endl;
			std::cout << "x : " << detectedPoints(0 ,0 )  << " y: "<< detectedPoints(0 ,1)  << std::endl;
			std::cout << "x : " << detectedPoints(1 ,0 )  << " y: "<< detectedPoints(1 ,1)  << std::endl;
			std::cout << "x : " << detectedPoints(2 ,0 )  << " y: "<< detectedPoints(2 ,1)  << std::endl;
			std::cout << "x : " << detectedPoints(3 ,0 )  << " y: "<< detectedPoints(3 ,1)  << std::endl;
			std::cout << "x : " << detectedPoints(4 ,0 )  << " y: "<< detectedPoints(4 ,1)  << std::endl;
			std::cout << "x : " << detectedPoints(5 ,0 )  << " y: "<< detectedPoints(5 ,1)  << std::endl;
			std::cout << "x : " << detectedPoints(6 ,0 )  << " y: "<< detectedPoints(6 ,1)  << std::endl;
			std::cout << "x : " << detectedPoints(7 ,0 )  << " y: "<< detectedPoints(7 ,1)  << std::endl;
			std::cout << "x : " << detectedPoints(8 ,0 )  << " y: "<< detectedPoints(8 ,1)  << std::endl;
			std::cout << "x : " << detectedPoints(9 ,0 )  << " y: "<< detectedPoints(9 ,1)  << std::endl;
			std::cout << "x : " << detectedPoints(10,0 )  << " y: "<< detectedPoints(10,1)  << std::endl;
			std::cout << "x : " << detectedPoints(11,0 )  << " y: "<< detectedPoints(11,1)  << std::endl;
			std::cout << "x : " << detectedPoints(12,0 )  << " y: "<< detectedPoints(12,1)  << std::endl;
			std::cout << "x : " << detectedPoints(13,0 )  << " y: "<< detectedPoints(13,1)  << std::endl;
			std::cout << "x : " << detectedPoints(14,0 )  << " y: "<< detectedPoints(14,1)  << std::endl;
			std::cout << "x : " << detectedPoints(15,0 )  << " y: "<< detectedPoints(15,1)  << std::endl;
			std::cout << "x : " << detectedPoints(16,0 )  << " y: "<< detectedPoints(16,1)  << std::endl;
			std::cout << "x : " << detectedPoints(17,0 )  << " y: "<< detectedPoints(17,1)  << std::endl;
			std::cout << "x : " << detectedPoints(18,0 )  << " y: "<< detectedPoints(18,1)  << std::endl;
			std::cout << "x : " << detectedPoints(19,0 )  << " y: "<< detectedPoints(19,1)  << std::endl;
			std::cout << "x : " << detectedPoints(20,0 )  << " y: "<< detectedPoints(20,1)  << std::endl;
			std::cout << "x : " << detectedPoints(21,0 )  << " y: "<< detectedPoints(21,1)  << std::endl;
			std::cout << "x : " << detectedPoints(22,0 )  << " y: "<< detectedPoints(22,1)  << std::endl;
			std::cout << "x : " << detectedPoints(23,0 )  << " y: "<< detectedPoints(23,1)  << std::endl;
			std::cout << "x : " << detectedPoints(24,0 )  << " y: "<< detectedPoints(24,1)  << std::endl;
			std::cout << "x : " << detectedPoints(25,0 )  << " y: "<< detectedPoints(25,1)  << std::endl;
			std::cout << "x : " << detectedPoints(26,0 )  << " y: "<< detectedPoints(26,1)  << std::endl;
			std::cout << "x : " << detectedPoints(27,0 )  << " y: "<< detectedPoints(27,1)  << std::endl;
			std::cout << "x : " << detectedPoints(28,0 )  << " y: "<< detectedPoints(28,1)  << std::endl;
			std::cout << "x : " << detectedPoints(29,0 )  << " y: "<< detectedPoints(29,1)  << std::endl;
			std::cout << "x : " << detectedPoints(30,0 )  << " y: "<< detectedPoints(30,1)  << std::endl;
			std::cout << "x : " << detectedPoints(31,0 )  << " y: "<< detectedPoints(31,1)  << std::endl;
			std::cout << "x : " << detectedPoints(681,0 )  << " y: "<<detectedPoints(681,1)  << std::endl;
	}
		else{
			std::cout << "not scanned " << std::endl;
		}
	}
	
	std::cout << "LaserScanner test ended..." << std::endl;
	
	return 0;
	
	}catch(int &ex) {
		std::cout << "exception: " << ex << std::endl;
	}
}