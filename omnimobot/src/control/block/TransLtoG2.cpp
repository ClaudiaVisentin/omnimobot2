#include <omnimobot/control/block/TransLtoG2.hpp>

using namespace omnimobot;
using namespace eeros::math;

TransLtoG2::TransLtoG2():
phi(0.0)
{
	inL.zero();
	outG.zero();
	RotLtoG.zero();
}


TransLtoG2::~TransLtoG2() { }


void TransLtoG2::run()
{
	inL = inLocal.getSignal().getValue();
	phi = inPhiGL.getSignal().getValue();
	
	double cosPhiGL = cos(phi);
	double sinPhiGL = sin(phi);
	
	RotLtoG(0,0) = cosPhiGL;
	RotLtoG(1,0) = sinPhiGL;

	RotLtoG(0,1) = -sinPhiGL;
	RotLtoG(1,1) = cosPhiGL;
	
	outG = RotLtoG * inL;
	
	outGlobal.getSignal().setValue(outG);
	outGlobal.getSignal().setTimestamp(inLocal.getSignal().getTimestamp());
	
}

