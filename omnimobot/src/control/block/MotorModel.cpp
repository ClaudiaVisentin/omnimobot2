#include <omnimobot/control/block/MotorModel.hpp>
#include <omnimobot/constants.hpp>

#include <iostream>
#include <fstream>

using namespace eeros::control;
using namespace omnimobot;

MotorModel::MotorModel( eeros::math::Matrix<6,6> gear, eeros::math::Matrix<6,6> invgear, Vector6 torqueLim) :
invGearGain(invgear),	
invMotConst(1.0/km),	
rotResistor(R),
gearAndMotConst(gear*km),
torqueSaturation(0.0-torqueLim,torqueLim)
{
	
	getOutVoltage().getSignal().setName("voltage");
	getInSpeed().getSignal().setName("Omega_ist");
	getInTorque().getSignal().setName("Torque_soll");

	torqueSaturation.getIn().connect(invGearGain.getOut());
	invMotConst.getIn().connect(torqueSaturation.getOut());
	rotResistor.getIn().connect(invMotConst.getOut());
	
	sum.getIn(0).connect(rotResistor.getOut());
	sum.getIn(1).connect(gearAndMotConst.getOut());	
}

MotorModel::~MotorModel()
{

}


void MotorModel::run() {
	invGearGain.run();
	torqueSaturation.run();
	invMotConst.run();
	rotResistor.run();
	gearAndMotConst.run();
	sum.run();
}

eeros::control::Input<Vector6>& MotorModel::getInTorque() {
	return invGearGain.getIn();
}

eeros::control::Input<Vector6>& MotorModel::getInSpeed() {
	return gearAndMotConst.getIn();
}

eeros::control::Output<Vector6>& MotorModel::getOutVoltage() {
	return sum.getOut();
}


