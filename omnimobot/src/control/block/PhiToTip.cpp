#include <omnimobot/control/block/PhiToTip.hpp>
#include <omnimobot/constants.hpp>

using namespace omnimobot;
using namespace eeros::math;
using namespace std;

PhiToTip::PhiToTip():
stablaenge(lengthOfStick)
{
	phiHall.zero();
	posRobot.zero();
	XYtip.zero();
}

PhiToTip::~PhiToTip() { }

void PhiToTip::run()
{
	phiHall = inPhixy.getSignal().getValue();

	posRobot = inPosRobot.getSignal().getValue();
	
	XYtip(0) = sin(phiHall(0)) * stablaenge + posRobot(0);
	XYtip(1) = sin(phiHall(1)) * stablaenge + posRobot(1);
	
	outTip.getSignal().setValue(XYtip);
	outTip.getSignal().setTimestamp(inPhixy.getSignal().getTimestamp()); 
	
}