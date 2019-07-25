#include <omnimobot/control/block/TransLtoG.hpp>

using namespace omnimobot;
using namespace eeros::math;

TransLtoG::TransLtoG():
phi(0.0)
{
	v_L.zero();
	veloGlobal.zero();
	RotLtoG.zero();
}


TransLtoG::~TransLtoG() { }


void TransLtoG::run()
{
	v_L = inV_L.getSignal().getValue();
	phi = inPhiGL.getSignal().getValue();
	
	double cosPhiGL = cos(phi);
	double sinPhiGL = sin(phi);
	
	RotLtoG(0,0) = cosPhiGL;
	RotLtoG(1,0) = sinPhiGL;

	RotLtoG(0,1) = -sinPhiGL;
	RotLtoG(1,1) = cosPhiGL;

	RotLtoG(2,2) = 1.0;
	
	veloGlobal = RotLtoG * v_L;
	
	outV_G.getSignal().setValue(veloGlobal);
	outV_G.getSignal().setTimestamp(inV_L.getSignal().getTimestamp());
	
}

