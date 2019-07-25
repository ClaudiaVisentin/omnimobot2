#include <omnimobot/control/block/FilterHall.hpp>
#include <omnimobot/constants.hpp>

omnimobot::FilterHall::FilterHall():
stickConnected(false)
{ 
	inHallValue.zero();
	outHallValue.zero();
	hallValuePrev.fill(2015.0);
}

omnimobot::FilterHall::~FilterHall() { }

void omnimobot::FilterHall::run()
{
	for ( int i = 0; i < 4; i++){
		inHallValue(i) = inHall.getSignal().getValue()(i)*bitFactor;
	
// 		if (inHallValue(i) < 2050) {
// 			inHallValue(i) = hallValuePrev(i);
// 		}
// 		
// 		if (i == 0 && inHallValue(i) > 2390) {
// 			inHallValue(i) = hallValuePrev(i);
// 		}
// 		
// 		if (i == 1 && inHallValue(i) > 2400) {
// 			inHallValue(i) = hallValuePrev(i);
// 		}
// 		
// 		if (i == 2 && inHallValue(i) > 2390) {
// 			inHallValue(i) = hallValuePrev(i);
// 		}
// 
// 		if (i == 3 && inHallValue(i) > 2380) {
// 			inHallValue(i) = hallValuePrev(i);
// 		}
// 
// 		hallValuePrev(i) = inHallValue(i);
	}
	
	// control is stick connected
	if(inHallValue(0) + inHallValue(1) +inHallValue(2)+ inHallValue(3) < 8400.0) {
		stickConnected = false;
	}
	else if(inHallValue(0) < 2067) { // if minimum value below 
		stickConnected = false;
	}
	else if(inHallValue(1) < 2065) { // if minimum value below
		stickConnected = false;
	}
	else if(inHallValue(2) < 2097) { // if minimum value below
		stickConnected = false;
	}
	else if(inHallValue(3) < 2077) { // if minimum value below
		stickConnected = false;
	}
	else {
		stickConnected = true;
	}
	
	
	outHallValue(0) = inHallValue(0)- 2020;//mean of value without stick           alt:2194.0;
	outHallValue(1) = inHallValue(1)- 2013;//               alt:2179.0;
	outHallValue(2) = inHallValue(2)- 2044;//               alt:2210.0;
	outHallValue(3) = inHallValue(3)- 2028;//               alt:2192.0;
	
	
	outHall0.getSignal().setValue(outHallValue(0));
	outHall0.getSignal().setTimestamp(inHall.getSignal().getTimestamp());
	
	outHall1.getSignal().setValue(outHallValue(1));
	outHall1.getSignal().setTimestamp(inHall.getSignal().getTimestamp());
	
	outHall2.getSignal().setValue(outHallValue(2));
	outHall2.getSignal().setTimestamp(inHall.getSignal().getTimestamp());
	
	outHall3.getSignal().setValue(outHallValue(3));
	outHall3.getSignal().setTimestamp(inHall.getSignal().getTimestamp());
}
