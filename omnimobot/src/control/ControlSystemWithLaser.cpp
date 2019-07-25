#include <omnimobot/control/ControlSystemWithLaser.hpp>
#include <constants.hpp>
#include <math.h>
#include <unistd.h>
#include <iostream>

#define JOYSTICK_DEVICE "/dev/input/js0"

using namespace eeros::control;
using namespace omnimobot;
using namespace eeros::math;


eeros::math::Matrix<6,6,double> ControlSystemWithLaser::gear = eeros::math::Matrix<6,6,double>({	gearWheel, 0.0,        0.0,        0.0,       0.0,        0.0, 
																						0.0,       gearWheel,  0.0,        0.0,       0.0,        0.0,
																						0.0,       0.0,        gearWheel,  0.0,       0.0,        0.0,
																						0.0,       0.0,        0.0,        gearSteer, 0.0,        0.0, 
																						0.0,       0.0,        0.0,        0.0,       gearSteer,  0.0,
																						0.0,       0.0,        0.0,        0.0,       0.0,        gearSteer}).transpose(); 

eeros::math::Matrix<6,6,double> ControlSystemWithLaser::invGear = eeros::math::Matrix<6,6,double>({	1.0/gearWheel,   0.0,           0.0,           0.0,           0.0,           0.0,
																							0.0,             1.0/gearWheel, 0.0,           0.0,           0.0,           0.0,
																							0.0,             0.0,           1.0/gearWheel, 0.0,           0.0,           0.0,
																							0.0,             0.0,           0.0,           1.0/gearSteer, 0.0,           0.0,
																							0.0,             0.0,           0.0,           0.0,           1.0/gearSteer, 0.0,
																							0.0,             0.0,           0.0,           0.0,           0.0,           1.0/gearSteer}).transpose();

																						
eeros::math::Matrix<6,6,double> ControlSystemWithLaser::inertia = eeros::math::Matrix<6,6,double>({	inertiaWheel,   0.0,          0.0,          0.0,          0.0,          0.0, 
																							0.0,            inertiaWheel, 0.0,          0.0,          0.0,          0.0,
																							0.0,            0.0,          inertiaWheel, 0.0,          0.0,          0.0,
																							0.0,            0.0,          0.0,          inertiaSteer, 0.0,          0.0, 
																							0.0,            0.0,          0.0,          0.0,          inertiaSteer, 0.0,
																							0.0,            0.0,          0.0,          0.0,          0.0,          inertiaSteer}).transpose();
																	
eeros::math::Matrix<6,1,double> ControlSystemWithLaser::torqueLim = eeros::math::Matrix<6,1,double>({torqueSaturation,
																							torqueSaturation,
																							torqueSaturation,
																							torqueSaturation,
																							torqueSaturation,
																							torqueSaturation	});

eeros::math::Matrix<6,1,double> ControlSystemWithLaser::speedLim = eeros::math::Matrix<6,1,double>({ speedLimWheel,
																							speedLimWheel,
																							speedLimWheel,
																							speedLimSteer,
																							speedLimSteer,
																							speedLimSteer    });
																							
eeros::math::Matrix<6,1,double> ControlSystemWithLaser::resetValue6 = eeros::math::Matrix<6,1,double>({ 0.0,
																							  0.0,
																							  0.0,
																							  0.0,
																							  0.0,
																							  0.0  });

eeros::math::Matrix<3,1,double> ControlSystemWithLaser::resetValue3 = eeros::math::Matrix<3,1,double>({ 0.0,
																							   0.0,
																							   0.0  });

eeros::math::Matrix<2,1,double> ControlSystemWithLaser::resetValue2 = eeros::math::Matrix<2,1,double>({ 0.0,
																							   0.0  });


ControlSystemWithLaser::ControlSystemWithLaser(double ts, TransitionLaserData& transLaserData) : 
enc0("q0"),
enc1("q1"),
enc2("q2"),
enc3("q3"),
enc4("q4"),
enc5("q5"),
pwmA0("pwmAaxis0"),
pwmA1("pwmAaxis1"),
pwmA2("pwmAaxis2"),
pwmA3("pwmAaxis3"),
pwmA4("pwmAaxis4"),
pwmA5("pwmAaxis5"),
pwmB0("pwmBaxis0"),
pwmB1("pwmBaxis1"),
pwmB2("pwmBaxis2"),
pwmB3("pwmBaxis3"),
pwmB4("pwmBaxis4"),
pwmB5("pwmBaxis5"),

gainDeltaQToQpoint(1.0/ts),

offsetEnc({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), 
// safetyStopValue({0.0, 0.0, 0.0}),

ishomed(false),
isReady(false),
checkstartdrive(false),

timedomain("Main time domain",ts,true),

robotControlBlock( gear, invGear, inertia, torqueLim, speedLim),
voltageToPwm(bridgeVoltage),
counter(100), 
swPendulumOn(0),
firstCounter(true),
joystick(JOYSTICK_DEVICE)
{
	muxEncIn.getIn(0).connect(enc0.getOut());
	muxEncIn.getIn(1).connect(enc1.getOut());
	muxEncIn.getIn(2).connect(enc2.getOut());
	muxEncIn.getIn(3).connect(enc3.getOut());
	muxEncIn.getIn(4).connect(enc4.getOut());
	muxEncIn.getIn(5).connect(enc5.getOut());
	
	gainDeltaQToQpoint.getIn().connect(muxEncIn.getOut());
	integratorQpoint.getIn().connect(gainDeltaQToQpoint.getOut());
	
	sumOffset.getIn(0).connect(offsetEnc.getOut());
	sumOffset.getIn(1).connect(integratorQpoint.getOut());
	
// 	demuxJoystick.getIn().connect(joystick.getOut());
	demuxJoystick.getIn().connect(joystick.getOutVGdes());

	muxVeloGlobal.getIn(0).connect(demuxJoystick.getOut(0));
	muxVeloGlobal.getIn(1).connect(demuxJoystick.getOut(1));

	jacobi.getInActualPosQ().connect(sumOffset.getOut());
	jacobi.getInOmega().connect(gainDeltaQToQpoint.getOut());
	
	integratorPhiPointglobalToPhiGL.getIn().connect(jacobi.getOutPhiGLd());
	
	transLtoS.getInVL().connect(jacobi.getOutVL());
	transLaserData.getInVeloActualSapostrophe().connect(transLtoS.getOutVSapostrophe());
	
	localToGlobalJacobi.getInPhiGL().connect(integratorPhiPointglobalToPhiGL.getOut());
	localToGlobalJacobi.getInVL().connect(jacobi.getOutVL());
	
	joystick.getInVGactual().connect(localToGlobalJacobi.getOutVG());
	
	integratorOdoPointToOdo.getIn().connect(localToGlobalJacobi.getOutVG());
	
	robotControlBlock.getInGlobalPhi().connect(integratorPhiPointglobalToPhiGL.getOut());
	robotControlBlock.getInPosActualQ().connect(sumOffset.getOut());

	robotControlBlock.getInSpeedActualQ().connect(gainDeltaQToQpoint.getOut());

	muxVeloGlobalxyPhi.getIn(0).connect(demuxJoystick.getOut(0));
	muxVeloGlobalxyPhi.getIn(1).connect(demuxJoystick.getOut(1));
	muxVeloGlobalxyPhi.getIn(2).connect(demuxJoystick.getOut(3));
	
	integratorVGlobalxyToPosGlobalxy.getIn().connect(muxVeloGlobal.getOut());
	
	localToGlobalHall.getInPhiGL().connect(integratorPhiPointglobalToPhiGL.getOut());
	localToGlobalHall.getInLocal().connect(hallSensorInput.getOutPhixy());
	
	pendulumControl.getInOdometry().connect(integratorOdoPointToOdo.getOut());
	pendulumControl.getInPosTip().connect(integratorVGlobalxyToPosGlobalxy.getOut());
	pendulumControl.getInPhixy().connect(localToGlobalHall.getOutGlobal());
	
	deMuxPendulumOut.getIn().connect(pendulumControl.getOutVelo());
	
	muxPendulumVeloDesired.getIn(0).connect(deMuxPendulumOut.getOut(0));
	muxPendulumVeloDesired.getIn(1).connect(deMuxPendulumOut.getOut(1));
	muxPendulumVeloDesired.getIn(2).connect(demuxJoystick.getOut(3));
	
	swPendulumOn.getIn(0).connect(muxVeloGlobalxyPhi.getOut());
	swPendulumOn.getIn(1).connect(muxPendulumVeloDesired.getOut());
// 	swPendulumOn.getIn(2).connect(safetyStopValue.getOut());

	transGtoS.getInPhiGL().connect(integratorPhiPointglobalToPhiGL.getOut());
	transGtoS.getInVG().connect(swPendulumOn.getOut());
	
	collisionAvoidance.getInColldataSapostrophe().connect(transLaserData.getOutCollDataSapostrophe());
	collisionAvoidance.getInisMoveCalc().connect(transLaserData.getOutIsObjectMoveCalc());
	collisionAvoidance.getInVeloDesSapostrophe().connect(transGtoS.getOutVSapostrophe());
	
	transStoG.getInPhiGL().connect(integratorPhiPointglobalToPhiGL.getOut());
	transStoG.getInVSapostrophe().connect(collisionAvoidance.getOutSafetyVeloSapostrophe());
	
	robotControlBlock.getInVeloGlobal().connect(transStoG.getOutVG());
	
	voltageToPwm.getInVoltage().connect(robotControlBlock.getOutVoltage());
	deMuxPwmAOut.getIn().connect(voltageToPwm.getOutPwmA());
	deMuxPwmBOut.getIn().connect(voltageToPwm.getOutPwmB());
	
	pwmA0.getIn().connect(deMuxPwmAOut.getOut(0));
	pwmA1.getIn().connect(deMuxPwmAOut.getOut(1));
	pwmA2.getIn().connect(deMuxPwmAOut.getOut(2));
	pwmA3.getIn().connect(deMuxPwmAOut.getOut(3));
	pwmA4.getIn().connect(deMuxPwmAOut.getOut(4));
	pwmA5.getIn().connect(deMuxPwmAOut.getOut(5));
	pwmB0.getIn().connect(deMuxPwmBOut.getOut(0));
	pwmB1.getIn().connect(deMuxPwmBOut.getOut(1));
	pwmB2.getIn().connect(deMuxPwmBOut.getOut(2));
	pwmB3.getIn().connect(deMuxPwmBOut.getOut(3));
	pwmB4.getIn().connect(deMuxPwmBOut.getOut(4));
	pwmB5.getIn().connect(deMuxPwmBOut.getOut(5));
	
	
	timedomain.addBlock(&enc0);
	timedomain.addBlock(&enc1);
	timedomain.addBlock(&enc2);
	timedomain.addBlock(&enc3);
	timedomain.addBlock(&enc4);
	timedomain.addBlock(&enc5);
	timedomain.addBlock(&hallSensorInput);
	timedomain.addBlock(&muxEncIn);
	timedomain.addBlock(&gainDeltaQToQpoint);
	timedomain.addBlock(&offsetEnc);
// 	timedomain.addBlock(&safetyStopValue);
	timedomain.addBlock(&integratorQpoint);
	timedomain.addBlock(&sumOffset);
	timedomain.addBlock(&joystick);
	timedomain.addBlock(&demuxJoystick);
	timedomain.addBlock(&muxVeloGlobal);
	timedomain.addBlock(&jacobi);
	timedomain.addBlock(&integratorPhiPointglobalToPhiGL);
	timedomain.addBlock(&transLtoS);
	timedomain.addBlock(transLaserData.getRunnableB());
	timedomain.addBlock(&localToGlobalJacobi);
	timedomain.addBlock(&integratorOdoPointToOdo);
	timedomain.addBlock(&muxVeloGlobalxyPhi);
	timedomain.addBlock(&integratorVGlobalxyToPosGlobalxy);
	timedomain.addBlock(&localToGlobalHall);
	timedomain.addBlock(&pendulumControl);
	timedomain.addBlock(&deMuxPendulumOut);
	timedomain.addBlock(&muxPendulumVeloDesired);
	timedomain.addBlock(&swPendulumOn);
	timedomain.addBlock(&transGtoS);
	timedomain.addBlock(&collisionAvoidance);	
	timedomain.addBlock(&transStoG);
	timedomain.addBlock(&robotControlBlock);
	timedomain.addBlock(&voltageToPwm);
	timedomain.addBlock(&deMuxPwmAOut);
	timedomain.addBlock(&deMuxPwmBOut);
	timedomain.addBlock(&pwmA0);
	timedomain.addBlock(&pwmA1);
	timedomain.addBlock(&pwmA2);
	timedomain.addBlock(&pwmA3);
	timedomain.addBlock(&pwmA4);
	timedomain.addBlock(&pwmA5);
	timedomain.addBlock(&pwmB0);
	timedomain.addBlock(&pwmB1);
	timedomain.addBlock(&pwmB2);
	timedomain.addBlock(&pwmB3);
	timedomain.addBlock(&pwmB4);
	timedomain.addBlock(&pwmB5);
	
}

ControlSystemWithLaser::~ControlSystemWithLaser()
{
	stop();
}


void ControlSystemWithLaser::start() {
	timedomain.start();
}

void ControlSystemWithLaser::stop() {
	timedomain.stop();
// 	timedomain.join();
}



void ControlSystemWithLaser::initReadyToDrive() {		
	checkStartDrive();
	
	integratorPhiPointglobalToPhiGL.setInitCondition(0.0);
	integratorPhiPointglobalToPhiGL.enable();

	integratorOdoPointToOdo.setInitCondition(resetValue3);
	integratorOdoPointToOdo.enable();

	integratorVGlobalxyToPosGlobalxy.setInitCondition(resetValue2);
	integratorVGlobalxyToPosGlobalxy.enable();
	
	if(!checkstartdrive){
		std::cout << " check for activated drive is bad! " << std::endl;
		isReady = false;
	}
	else {
		// Homing Switch
		robotControlBlock.swSafetystopOn.switchToInput(0);
		robotControlBlock.homingBlock.swSteer1.switchToInput(0);
		robotControlBlock.homingBlock.swSteer2.switchToInput(0);
		robotControlBlock.homingBlock.swSteer3.switchToInput(0);
		robotControlBlock.homingBlock.swWeel1.switchToInput(0);
		robotControlBlock.homingBlock.swWeel2.switchToInput(0);
		robotControlBlock.homingBlock.swWeel3.switchToInput(0);
		isReady = true;
	}
}



void ControlSystemWithLaser::initReadyToPendulum()
{
	// pos in local coordinate system x,y
	integratorVGlobalxyToPosGlobalxy.setInitCondition(resetValue2);
	integratorVGlobalxyToPosGlobalxy.enable();
	// odometrie in local coordinate system x,y,theta
	integratorOdoPointToOdo.setInitCondition(resetValue3);
	integratorOdoPointToOdo.enable();
	// pos Tip x,y
	pendulumControl.integrator.setInitCondition(resetValue2);
	pendulumControl.integrator.enable();
	usleep(2100);
	swPendulumOn.switchToInput(1);
}



void ControlSystemWithLaser::checkStartDrive(){
	robotControlBlock.integrator.enable();				
	robotControlBlock.setHomingSpeed({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
	robotControlBlock.swHomingOn.switchToInput(1); // 1: inv_Jacobi noch nicht eingeschaltet (Soll speed ist immernoch Homingspeed) 0: inv Jacobi eingeschaltet
		
	usleep(2100); 
	
	Vector6 encPos;
	encPos = sumOffset.getOut().getSignal().getValue();
	
	robotControlBlock.integrator.setInitCondition(encPos);
	
	usleep(2100);

	Vector6 intValue;
	intValue = robotControlBlock.integrator.getOut().getSignal().getValue();
	
	Vector6 diff;
	diff = encPos - intValue;
	
	double d = 0.0;

	// Check difference
	for( int i = 0; i < 6; i++){
		if(diff(i) < 0){
			d += -diff(i);			
		}
		else {
			d += diff(i);
		}
	}
	
	
	if(d > 0.001){
		std::cout  << "d: " << d << std::endl;
		checkstartdrive = false;
	}
	else {
		checkstartdrive = true;
	}
}



void ControlSystemWithLaser::setHomingToTrue(){
	ishomed = true;
}


bool ControlSystemWithLaser::isaxisHomed(){
	return ishomed;
}


bool ControlSystemWithLaser::isreadyToDrive(){
	return isReady;
}



void ControlSystemWithLaser::initHoming() {
	if(ishomed) return;
	
	robotControlBlock.swHomingOn.switchToInput(1); // inv_Jacobi  nicht eingeschaltet (Soll speed ist immernoch Homingspeed)
	
	allAxisStopped();
	
	robotControlBlock.integrator.setInitCondition(resetValue6);		// Integrator reset
	robotControlBlock.integrator.enable();
	integratorQpoint.setInitCondition(resetValue6);                       // Encoder integrator reset
	integratorQpoint.enable();
	
};



void ControlSystemWithLaser::allAxisStopped()
{
	swPendulumOn.switchToInput(0);
	
	robotControlBlock.homingBlock.swSteer1.switchToInput(1);
	robotControlBlock.homingBlock.swSteer2.switchToInput(1);
	robotControlBlock.homingBlock.swSteer3.switchToInput(1);
	robotControlBlock.homingBlock.swWeel1.switchToInput(1);
	robotControlBlock.homingBlock.swWeel2.switchToInput(1);
	robotControlBlock.homingBlock.swWeel3.switchToInput(1);
	
	integratorPhiPointglobalToPhiGL.disable();
	robotControlBlock.integrator.disable();
	
	integratorVGlobalxyToPosGlobalxy.disable();
	integratorOdoPointToOdo.disable();
	pendulumControl.integrator.disable();
}



void ControlSystemWithLaser::setWheelToHoming(int steer)
{

	if(steer == steer1){	// Homing Rad1
		robotControlBlock.integrator.setInitCondition(resetValue6);
		robotControlBlock.homingBlock.swSteer1.switchToInput(0); // Rad1 steer get torque 
	}
	else if(steer == steer2){ // Homing Rad2
		robotControlBlock.integrator.setInitCondition(resetValue6);
		robotControlBlock.homingBlock.swSteer2.switchToInput(0);
	}
	else{ // Homing Rad3
		robotControlBlock.integrator.setInitCondition(resetValue6);
		robotControlBlock.homingBlock.swSteer3.switchToInput(0);
	}
}
