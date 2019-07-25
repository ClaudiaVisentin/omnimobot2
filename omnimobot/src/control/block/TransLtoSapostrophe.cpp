#include <omnimobot/control/block/TransLtoSapostrophe.hpp>
#include <omnimobot/constants.hpp>

using namespace omnimobot;
using namespace eeros::math;

TransLtoSapostrophe::TransLtoSapostrophe():
cosPhi(0.0),
sinPhi(0.0),
phi(0.0)
{ 
	v_L.zero();
	v_Sap.zero();
	RotLtoSap.zero();
	
	cosPhi = cos(pi); // only for omniderectional drive (see doku)
	sinPhi = sin(pi);
}

TransLtoSapostrophe::~TransLtoSapostrophe() { }

void TransLtoSapostrophe::run()
{
	v_L = inV_L.getSignal().getValue();
	
	RotLtoSap(0,0) = cosPhi;
	RotLtoSap(1,0) = -sinPhi;

	RotLtoSap(0,1) = sinPhi;
	RotLtoSap(1,1) = cosPhi;

	RotLtoSap(2,2) = 1.0;
	
	v_Sap = RotLtoSap * v_L;
	
	outV_Sapostrophe.getSignal().setValue(v_Sap);
	outV_Sapostrophe.getSignal().setTimestamp(inV_L.getSignal().getTimestamp());
}

