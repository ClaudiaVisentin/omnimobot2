#include <omnimobot/control/block/LaserScannerblock.hpp>
#include <iostream>

using namespace omnimobot;
using namespace eeros::math;


LaserScannerblock::LaserScannerblock(std::string dev):
laserScanner(startStep, endStep)
{ 
	colldata.zero();
	while(!laserScanner.openScanner(dev.c_str())) {		
		sleep(1);
	};
}

LaserScannerblock::~LaserScannerblock() { 
	laserScanner.closeScanner();
}

void LaserScannerblock::run()
{
	try {
		
	if(laserScanner.scan()){
		int dataSize = laserScanner.getOutDataSize();
		DataMatrix distanceAndAngle = laserScanner.getScannData();
				
		if(collDedection.calcCollisionsData(distanceAndAngle, dataSize)){
			colldata = collDedection.getCollisionsData();
		}
		else {
			colldata.zero();
			std::cout << "not CollDatta " << std::endl;
		}
	}
	else{
		std::cout << "not scanned " << std::endl;
	}
	
	}catch(int &ex) {
		std::cout << "exception in LaserScannerblock: " << ex << std::endl;
	}
	
	outColldata.getSignal().setValue(colldata);
	uint64_t ts = eeros::System::getTimeNs();
	outColldata.getSignal().setTimestamp(ts);
}
