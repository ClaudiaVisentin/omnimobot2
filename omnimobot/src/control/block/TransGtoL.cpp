#include <omnimobot/control/block/TransGtoL.hpp>

using namespace omnimobot;
using namespace eeros::math;

TransGtoL::TransGtoL():
phi(0.0)
{ 
	v_G.zero();
	veloLokal.zero();
	RotGtoL.zero();
}

TransGtoL::~TransGtoL() { }

void TransGtoL::run()
{
	
	v_G = inV_G.getSignal().getValue();
	phi = inPhiGL.getSignal().getValue();
	
	double cosPhiGL = cos(phi);
	double sinPhiGL = sin(phi);
	
	RotGtoL(0,0) = cosPhiGL;
	RotGtoL(1,0) = -sinPhiGL;

	RotGtoL(0,1) = sinPhiGL;
	RotGtoL(1,1) = cosPhiGL;

	RotGtoL(2,2) = 1.0;
	
	veloLokal = RotGtoL * v_G;
	
	outV_L.getSignal().setValue(veloLokal);
	outV_L.getSignal().setTimestamp(inV_G.getSignal().getTimestamp());
	
}

