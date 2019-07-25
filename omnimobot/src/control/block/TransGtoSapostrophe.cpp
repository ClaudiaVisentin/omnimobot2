#include <omnimobot/control/block/TransGtoSapostrophe.hpp>


using namespace omnimobot;
using namespace eeros::math;

TransGtoSapostrophe::TransGtoSapostrophe():
phi(0.0)
{ 
	v_G.zero();
	veloSap.zero();
	RotGtoSap.zero();
}

TransGtoSapostrophe::~TransGtoSapostrophe() { }

void TransGtoSapostrophe::run()
{
	
	v_G = inV_G.getSignal().getValue();
	phi = inPhiGL.getSignal().getValue();
	
	double cosPhiGLPi = cos(pi+phi);
	double sinPhiGLPi = sin(pi+phi);
	
	RotGtoSap(0,0) = cosPhiGLPi;
	RotGtoSap(1,0) = -sinPhiGLPi;

	RotGtoSap(0,1) = sinPhiGLPi;
	RotGtoSap(1,1) = cosPhiGLPi;

	RotGtoSap(2,2) = 1.0;
	
	veloSap = RotGtoSap * v_G;
	
	outV_Sapo.getSignal().setValue(veloSap);
	outV_Sapo.getSignal().setTimestamp(inV_G.getSignal().getTimestamp());
	
}

