#include <omnimobot/control/block/AccPhiToAccxy.hpp>
#include <omnimobot/constants.hpp>

using namespace omnimobot;
using namespace eeros::math;
using namespace std;

AccPhiToAccxy::AccPhiToAccxy():
posSchwerpunkt(posGravityPoint),
traegheitStab(inertiaStick),
masseStab(mStick),
variableR(rFactor)
{
	phiHall.zero();
	AccPhi.zero();
	Accxy.zero();
}

AccPhiToAccxy::~AccPhiToAccxy() { }

void AccPhiToAccxy::run()
{
	phiHall = inPhixy.getSignal().getValue();
	
	AccPhi = inAccPhi.getSignal().getValue();
	
	Accxy(0) = -1.0*(AccPhi(0) * (posSchwerpunkt*posSchwerpunkt + variableR*variableR) / posSchwerpunkt / cos(phiHall(0))) + gravity * tan(phiHall(0));  
	Accxy(1) = -1.0*(AccPhi(1) * (posSchwerpunkt*posSchwerpunkt + variableR*variableR) / posSchwerpunkt / cos(phiHall(1))) + gravity * tan(phiHall(1));
	
	outAccxy.getSignal().setValue(Accxy);
	outAccxy.getSignal().setTimestamp(inAccPhi.getSignal().getTimestamp()); 
	
}