#include <omnimobot/control/block/AccTipToPhi.hpp>
#include <omnimobot/constants.hpp>

#include <cmath>

using namespace omnimobot;
using namespace eeros::math;
using namespace std;

AccTipToPhi::AccTipToPhi() 
{ 
	accTip.zero();
	phisoll.zero();
}

AccTipToPhi::~AccTipToPhi() { }

void AccTipToPhi::run()
{
	
	accTip = inAccTip.getSignal().getValue();
	
	phisoll(0) = atan2(accTip(0),gravity);
	phisoll(1) = atan2(accTip(1),gravity); 
	
	outPhiSoll.getSignal().setValue(phisoll);
	outPhiSoll.getSignal().setTimestamp(inAccTip.getSignal().getTimestamp()); 
	
}