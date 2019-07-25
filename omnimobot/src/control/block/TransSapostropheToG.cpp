#include <omnimobot/control/block/TransSapostropheToG.hpp>


using namespace omnimobot;
using namespace eeros::math;

TransSapostropheToG::TransSapostropheToG():
phi(0.0)
{ 
	v_G.zero();
	veloSap.zero();
	RotSaptoG.zero();
}

TransSapostropheToG::~TransSapostropheToG() { }

void TransSapostropheToG::run()
{
	
	veloSap = inV_Sapo.getSignal().getValue();
	phi = inPhiGL.getSignal().getValue();
	
	double cosPhiGLPi = cos(pi+phi);
	double sinPhiGLPi = sin(pi+phi);
	
	RotSaptoG(0,0) = cosPhiGLPi;
	RotSaptoG(1,0) = sinPhiGLPi;

	RotSaptoG(0,1) = -sinPhiGLPi;
	RotSaptoG(1,1) = cosPhiGLPi;

	RotSaptoG(2,2) = 1.0;
	
	v_G = RotSaptoG * veloSap;
	
	outV_G.getSignal().setValue(v_G);
	outV_G.getSignal().setTimestamp(inV_Sapo.getSignal().getTimestamp());
}

