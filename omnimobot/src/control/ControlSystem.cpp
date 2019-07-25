#include <omnimobot/control/ControlSystem.hpp>
#include <constants.hpp>
#include <math.h>
#include <unistd.h>
#include <iostream>

#define JOYSTICK_DEVICE "/dev/input/js0"

using namespace eeros::control;
using namespace omnimobot;
using namespace eeros::math;


eeros::math::Matrix<6,6,double> ControlSystem::gear = eeros::math::Matrix<6,6,double>({	gearWheel, 0.0,        0.0,        0.0,       0.0,        0.0, // (18) gear Antrieb
																						0.0,       gearWheel,  0.0,        0.0,       0.0,        0.0,
																						0.0,       0.0,        gearWheel,  0.0,       0.0,        0.0,
																						0.0,       0.0,        0.0,        gearSteer, 0.0,        0.0, // (36.75) gear Lenkung
																						0.0,       0.0,        0.0,        0.0,       gearSteer,  0.0,
																						0.0,       0.0,        0.0,        0.0,       0.0,        gearSteer}).transpose(); // transponiert, da die Matrix 1.Spalte dann 2. Spalte fühlt

eeros::math::Matrix<6,6,double> ControlSystem::invGear = eeros::math::Matrix<6,6,double>({	1.0/gearWheel,   0.0,           0.0,           0.0,           0.0,           0.0,
																							0.0,             1.0/gearWheel, 0.0,           0.0,           0.0,           0.0,
																							0.0,             0.0,           1.0/gearWheel, 0.0,           0.0,           0.0,
																							0.0,             0.0,           0.0,           1.0/gearSteer, 0.0,           0.0,
																							0.0,             0.0,           0.0,           0.0,           1.0/gearSteer, 0.0,
																							0.0,             0.0,           0.0,           0.0,           0.0,           1.0/gearSteer}).transpose();

																						
eeros::math::Matrix<6,6,double> ControlSystem::inertia = eeros::math::Matrix<6,6,double>({	inertiaWheel,   0.0,          0.0,          0.0,          0.0,          0.0, 
																							0.0,            inertiaWheel, 0.0,          0.0,          0.0,          0.0,
																							0.0,            0.0,          inertiaWheel, 0.0,          0.0,          0.0,
																							0.0,            0.0,          0.0,          inertiaSteer, 0.0,          0.0, 
																							0.0,            0.0,          0.0,          0.0,          inertiaSteer, 0.0,
																							0.0,            0.0,          0.0,          0.0,          0.0,          inertiaSteer}).transpose();
																			
eeros::math::Matrix<6,1,double> ControlSystem::torqueLim = eeros::math::Matrix<6,1,double>({torqueSaturation,
																							torqueSaturation,
																							torqueSaturation,
																							torqueSaturation,
																							torqueSaturation,
																							torqueSaturation	});

eeros::math::Matrix<6,1,double> ControlSystem::speedLim = eeros::math::Matrix<6,1,double>({ speedLimWheel,//20.0,
																							speedLimWheel,//20.0,
																							speedLimWheel,//20.0,
																							speedLimSteer,//10.36,
																							speedLimSteer,//10.36,
																							speedLimSteer  });//10.36	});
																							
eeros::math::Matrix<6,1,double> ControlSystem::resetValue6 = eeros::math::Matrix<6,1,double>({ 0.0,
																							  0.0,
																							  0.0,
																							  0.0,
																							  0.0,
																							  0.0  });

eeros::math::Matrix<3,1,double> ControlSystem::resetValue3 = eeros::math::Matrix<3,1,double>({ 0.0,
																							   0.0,
																							   0.0  });

eeros::math::Matrix<2,1,double> ControlSystem::resetValue2 = eeros::math::Matrix<2,1,double>({ 0.0,
																							   0.0  });


ControlSystem::ControlSystem(double ts) : 
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

offsetEnc({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), //[rad]

ishomed(false),
isPendulum(false),
isReady(false),
ischeckstartdrive(false),
ischeckstartPendulum(false),

timedomain("Main time domain",ts,true), 		// TODO make realtime

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
	
// 	integratorPhiPointglobalToPhiGL.getIn().connect(demuxJoystick.getOut(3));
	integratorPhiPointglobalToPhiGL.getIn().connect(jacobi.getOutPhiGLd());
	
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
	
	robotControlBlock.getInVeloGlobal().connect(swPendulumOn.getOut());
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
	
	measuringPhiHall.getIn().connect(localToGlobalHall.getOutGlobal());
	measuringPhiHall.getIn().connect(hallSensorInput.getOutPhixy());
	measuringPhides.getIn().connect(pendulumControl.aTip_To_Phi.getOutPhiSoll());
	measuringVeloIn.getIn().connect(muxVeloGlobal.getOut());
	measuringVeloOut.getIn().connect(localToGlobalJacobi.getOutVG());
	
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
	timedomain.addBlock(&integratorQpoint);
	timedomain.addBlock(&sumOffset);
	timedomain.addBlock(&joystick);
	timedomain.addBlock(&demuxJoystick);
	timedomain.addBlock(&muxVeloGlobal);
	timedomain.addBlock(&jacobi);
	timedomain.addBlock(&integratorPhiPointglobalToPhiGL);
	timedomain.addBlock(&localToGlobalJacobi);
	timedomain.addBlock(&integratorOdoPointToOdo);
	timedomain.addBlock(&muxVeloGlobalxyPhi);
	timedomain.addBlock(&integratorVGlobalxyToPosGlobalxy);
	timedomain.addBlock(&localToGlobalHall);
	timedomain.addBlock(&pendulumControl);
	timedomain.addBlock(&deMuxPendulumOut);
	timedomain.addBlock(&muxPendulumVeloDesired);
	timedomain.addBlock(&swPendulumOn);
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
	
	timedomain.addBlock(&measuringPhiHall);
	timedomain.addBlock(&measuringPhides);
	timedomain.addBlock(&measuringVeloIn);
	timedomain.addBlock(&measuringVeloOut);
	
//		std::cout << "CS Thread ID: " << timedomain.getId()<< std::endl;
	
}

ControlSystem::~ControlSystem()
{
	stop();
}

void ControlSystem::start() {
	timedomain.start();
}

void ControlSystem::stop() {
	timedomain.stop();
// 	timedomain.join();
}



void ControlSystem::initReadyToDrive() {	
	checkStartDrive();
	// phi
	integratorPhiPointglobalToPhiGL.setInitCondition(0.0);
	integratorPhiPointglobalToPhiGL.enable();
	// odometrie in local coordinate system x,y,theta
	integratorOdoPointToOdo.setInitCondition(resetValue3);
	integratorOdoPointToOdo.enable();

	integratorVGlobalxyToPosGlobalxy.setInitCondition(resetValue2);
	integratorVGlobalxyToPosGlobalxy.enable();
	
	pendulumControl.integrator.setInitCondition(resetValue2);
	pendulumControl.integrator.enable();
	
	if(!ischeckstartdrive){
		std::cout << " check for activated drive is bad! " << std::endl;
		isReady = false;
	}
	else {
		std::cout << " out intergator: "<<pendulumControl.integrator.getOut().getSignal().getValue()(0) << std::endl;
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



void ControlSystem::initReadyToPendulum()
{
	checkStartPendulum();
	
	if(!ischeckstartPendulum){
		std::cout << " check for activated drive is bad! " << std::endl;
		isPendulum = false;
	}
	else {
		
		// velo cs x,y
		pendulumControl.integrator.setInitCondition(resetValue2);
		pendulumControl.integrator.enable();
		usleep(2100); 
		swPendulumOn.switchToInput(1);
		isPendulum = true;
	}
}



void ControlSystem::checkStartPendulum()
{
	integratorOdoPointToOdo.setInitCondition(resetValue3);
// 	integratorVGlobalxyToPosGlobalxy.enable();
	
	usleep(2100);
	
	Vector<3> actualOdo;
	actualOdo = integratorOdoPointToOdo.getOut().getSignal().getValue();
	
	Vector<2> desiredPos;
	desiredPos(0) = actualOdo(0);
	desiredPos(1) = actualOdo(1);
	
	integratorVGlobalxyToPosGlobalxy.setInitCondition(desiredPos);
	
	usleep(2100);

	Vector<2> intValue;
	intValue = integratorVGlobalxyToPosGlobalxy.getOut().getSignal().getValue();

	Vector<2> diff;
	diff = desiredPos - intValue;
	
	double d = 0.0;

	// Check difference
	for( int i = 0; i < 2; i++){
		if(diff(i) < 0){
			d += -diff(i);			
		}
		else {
			d += diff(i);
		}
	}

	if(d > 0.001){
		ischeckstartPendulum = false;
	}
	else {
		ischeckstartPendulum = true;
	}
}



void ControlSystem::checkStartDrive(){
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
		ischeckstartdrive = false;
	}
	else {
		ischeckstartdrive = true;
	}
	
}

void ControlSystem::setHomingToTrue(){
	ishomed = true;
}


bool ControlSystem::isaxisHomed(){
	return ishomed;
}


bool ControlSystem::isReadyToDrive(){
	return isReady;
}

bool ControlSystem::isReadyToPendulum(){
	return isPendulum;
}



void ControlSystem::initHoming() {
	if(ishomed) return;
	
	robotControlBlock.swHomingOn.switchToInput(1); // inv_Jacobi  nicht eingeschaltet (Soll speed ist immernoch Homingspeed)
	
	allAxisStopped();
	
	robotControlBlock.integrator.setInitCondition(resetValue6);		// Integrator reset
	robotControlBlock.integrator.enable();
	integratorQpoint.setInitCondition(resetValue6);                       // Encoder integrator reset
	integratorQpoint.enable();
	
};



void ControlSystem::allAxisStopped()
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



void ControlSystem::setWheelToHoming(int steer)
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
