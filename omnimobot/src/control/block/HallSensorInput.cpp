#include <omnimobot/control/block/HallSensorInput.hpp>
#include <omnimobot/constants.hpp>

#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>

using namespace omnimobot;
using namespace eeros::control;


HallSensorInput::HallSensorInput():
hall0("hall_0"),
hall1("hall_1"),
hall2("hall_2"),
hall3("hall_3"),
gainWithFactor(factor),
calibration({errorInx,errorIny})
{
	muxHallValues.getIn(0).connect(hall0.getOut());
	muxHallValues.getIn(1).connect(hall1.getOut());
	muxHallValues.getIn(2).connect(hall2.getOut());
	muxHallValues.getIn(3).connect(hall3.getOut());
	
	filterHall.getInHall().connect(muxHallValues.getOut());
	
	sumHall0AndHall2.getIn(0).connect(filterHall.getOutHall0());
	sumHall0AndHall2.negateInput(1);
	sumHall0AndHall2.getIn(1).connect(filterHall.getOutHall2());
	
	sumHall3AndHall1.getIn(0).connect(filterHall.getOutHall1());
	sumHall3AndHall1.negateInput(1);
	sumHall3AndHall1.getIn(1).connect(filterHall.getOutHall3());
	
	muxValueOfSums.getIn(0).connect(sumHall0AndHall2.getOut());
	muxValueOfSums.getIn(1).connect(sumHall3AndHall1.getOut());
	
	gainWithFactor.getIn().connect(muxValueOfSums.getOut());
	
	sumcalibration.getIn(0).connect(gainWithFactor.getOut());
	sumcalibration.getIn(1).connect(calibration.getOut());
	
}


HallSensorInput::~HallSensorInput(){ }


void HallSensorInput::run()
{
	hall0.run();
	hall1.run();
	hall2.run();
	hall3.run();
	
	muxHallValues.run();
	filterHall.run();
	
	sumHall0AndHall2.run();
	sumHall3AndHall1.run();
	muxValueOfSums.run();
	gainWithFactor.run();
	calibration.run();
	sumcalibration.run();
	
}


Output< eeros::math::Vector<2> >& HallSensorInput::getOutPhixy()
{
	return sumcalibration.getOut();
}


bool HallSensorInput::isStickConnected()
{
	return filterHall.isStickConnected();
}
