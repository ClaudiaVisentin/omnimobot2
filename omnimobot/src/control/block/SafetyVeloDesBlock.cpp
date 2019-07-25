#include <omnimobot/control/block/SafetyVeloDesBlock.hpp>
#include <iostream>

using namespace omnimobot;
using namespace eeros::math;


SafetyVeloDesBlock::SafetyVeloDesBlock()
{ 
	colldata.zero();
	safetyVelo.zero();
	veloDes.zero();
}

SafetyVeloDesBlock::~SafetyVeloDesBlock() { }

void SafetyVeloDesBlock::run()
{
	veloDes = inVeloDesired.getSignal().getValue();
	colldata = inColldata.getSignal().getValue();
	

// 	safetyVeloDefine.setCalcObjectMove(inIsMoveCalc.getSignal().getValue());
	
	if (safetyVeloDefine.calcSafetyVeloDes(colldata,veloDes)) {
		
		safetyVelo = safetyVeloDefine.getSafetyVelo();
	}
	else {
		safetyVelo.zero();
		std::cout << "error in calcSafetyVeloDes " << std::endl;
	}
	
	outSafetyVelo.getSignal().setValue(safetyVelo);
	outSafetyVelo.getSignal().setTimestamp(inVeloDesired.getSignal().getTimestamp());
}
