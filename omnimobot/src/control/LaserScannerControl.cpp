#include <omnimobot/control/LaserScannerControl.hpp>

#include <math.h>
#include <unistd.h>
#include <iostream>

#define LASERSCANNER_DEVICE "/dev/ttyPSC1"

using namespace eeros::control;
using namespace omnimobot;
using namespace eeros::math;


LaserScannerControl::LaserScannerControl(double ts) : 
laserBlock(LASERSCANNER_DEVICE),
timedomain("laser time domain",ts,false)
{
	transDataBlock.getInCollDataSapostrophe().connect(laserBlock.getOutColldata());
	
    timedomain.addBlock(&laserBlock);
	timedomain.addBlock(transDataBlock.getRunnableA());
}

LaserScannerControl::~LaserScannerControl()
{
	stop();
}


void LaserScannerControl::start() {
	timedomain.start();
}

void LaserScannerControl::stop() {
	timedomain.stop();
// 	timedomain.join();
}
