#include <omnimobot/control/block/RobotControl.hpp>
#include <omnimobot/constants.hpp>

using namespace eeros::math;
using namespace eeros::control;
using namespace omnimobot;

RobotControl::RobotControl( Matrix<6,6> gear, Matrix<6,6> invgear, Matrix<6,6> inertia, Vector6 torqueLim, Vector6 speedLim):
inPosActualQ(1.0),
inSpeedActualQ(1.0),
homingspeedsoll({0.0, 0.0, 0.0, homingspeed, homingspeed, homingspeed}), // desired Homingspeed 1[rad]
swHomingOn(1),
swSafetystopOn(0),
safetyStopValue({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
kpGain(kp),
speedSaturation(0.0-speedLim,speedLim),
kdGain(kd),
inertiaRobot(inertia),
motModel(gear, invgear, torqueLim)
{
	getInVeloGlobal().getSignal().setName("Velocity in Local");
	getInSpeedActualQ().getSignal().setName("Omega_Actual");
	getInPosActualQ().getSignal().setName("Pos_Actual");
	getOutVoltage().getSignal().setName("Voltage");
	
	posInDeMux.getIn().connect(inPosActualQ.getOut());
	
	posSteerInMux.getIn(0).connect(posInDeMux.getOut(3));
	posSteerInMux.getIn(1).connect(posInDeMux.getOut(4));
	posSteerInMux.getIn(2).connect(posInDeMux.getOut(5));
	
	invjacobi.getInPhiSteer().connect(posSteerInMux.getOut());

	swHomingOn.getIn(0).connect(invjacobi.getOutOmega());
	swHomingOn.getIn(1).connect(homingspeedsoll.getOut());
	integrator.getIn().connect(swHomingOn.getOut());
	sum1.getIn(0).connect(integrator.getOut());
	sum1.negateInput(1);
	sum1.getIn(1).connect(inPosActualQ.getOut());
	kpGain.getIn().connect(sum1.getOut());
	sum2.getIn(0).connect(swHomingOn.getOut());
	sum2.getIn(1).connect(kpGain.getOut());
	speedSaturation.getIn().connect(sum2.getOut());
	swSafetystopOn.getIn(0).connect(speedSaturation.getOut());
	swSafetystopOn.getIn(1).connect(safetyStopValue.getOut());
	sum3.getIn(0).connect(swSafetystopOn.getOut());
	sum3.negateInput(1);
	sum3.getIn(1).connect(inSpeedActualQ.getOut());
	kdGain.getIn().connect(sum3.getOut());
	inertiaRobot.getIn().connect(kdGain.getOut());
	
	homingBlock.getInTorque().connect(inertiaRobot.getOut());
	motModel.getInTorque().connect(homingBlock.getOutHomingTorque());
	motModel.getInSpeed().connect(inSpeedActualQ.getOut());

}


#include <chrono>
using clk = std::chrono::high_resolution_clock;
using duration = std::chrono::duration<double>;

void RobotControl::run() {
	inPosActualQ.run();
	inSpeedActualQ.run();
	posInDeMux.run();
	posSteerInMux.run();
	invjacobi.run();
	homingspeedsoll.run();
	swHomingOn.run();
	integrator.run();
	sum1.run();
	kpGain.run();
	sum2.run();
	speedSaturation.run();
	safetyStopValue.run();
	swSafetystopOn.run();
	sum3.run();
	kdGain.run();
	inertiaRobot.run();
	homingBlock.run();
	motModel.run();
	
}

RobotControl::~RobotControl()
{

}


void RobotControl::setHomingSpeed(Vector6 u)
{
	homingspeedsoll.setValue(u);
}


eeros::control::Input<Vector3>& RobotControl::getInVeloGlobal() {
	return invjacobi.getInVglobal();
}

eeros::control::Input<Vector6>& RobotControl::getInSpeedActualQ() {
	return inSpeedActualQ.getIn();
}

eeros::control::Input<Vector6>& RobotControl::getInPosActualQ() {
	return inPosActualQ.getIn();
}

eeros::control::Output<Vector6>& RobotControl::getOutVoltage() {
	return motModel.getOutVoltage();
}

eeros::control::Input< >& RobotControl::getInGlobalPhi()
{
	return invjacobi.getInPhiGlobal();
}
