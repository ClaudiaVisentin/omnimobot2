#include <omnimobot/control/ControlSystemWithJacobi.hpp>
#include <constants.hpp>
#include <math.h>
#include <unistd.h>
#include <iostream>

#define JOYSTICK_DEVICE "/dev/input/js0"

using namespace eeros::control;
using namespace omnimobot;
using namespace eeros::math;


eeros::math::Matrix<6,6,double> ControlSystemWithJacobi::gear = eeros::math::Matrix<6,6,double>({	gearWheel, 0.0,        0.0,        0.0,       0.0,        0.0, // (18) gear Antrieb
																						0.0,       gearWheel,  0.0,        0.0,       0.0,        0.0,
																						0.0,       0.0,        gearWheel,  0.0,       0.0,        0.0,
																						0.0,       0.0,        0.0,        gearSteer, 0.0,        0.0, // (36.75) gear Lenkung
																						0.0,       0.0,        0.0,        0.0,       gearSteer,  0.0,
																						0.0,       0.0,        0.0,        0.0,       0.0,        gearSteer}).transpose(); // transponiert, da die Matrix 1.Spalte dann 2. Spalte fühlt

eeros::math::Matrix<6,6,double> ControlSystemWithJacobi::invGear = eeros::math::Matrix<6,6,double>({	1.0/gearWheel,   0.0,           0.0,           0.0,           0.0,           0.0,
																							0.0,             1.0/gearWheel, 0.0,           0.0,           0.0,           0.0,
																							0.0,             0.0,           1.0/gearWheel, 0.0,           0.0,           0.0,
																							0.0,             0.0,           0.0,           1.0/gearSteer, 0.0,           0.0,
																							0.0,             0.0,           0.0,           0.0,           1.0/gearSteer, 0.0,
																							0.0,             0.0,           0.0,           0.0,           0.0,           1.0/gearSteer}).transpose();

																						
eeros::math::Matrix<6,6,double> ControlSystemWithJacobi::inertia = eeros::math::Matrix<6,6,double>({	inertiaWheel,   0.0,          0.0,          0.0,          0.0,          0.0, 
																							0.0,            inertiaWheel, 0.0,          0.0,          0.0,          0.0,
																							0.0,            0.0,          inertiaWheel, 0.0,          0.0,          0.0,
																							0.0,            0.0,          0.0,          inertiaSteer, 0.0,          0.0, 
																							0.0,            0.0,          0.0,          0.0,          inertiaSteer, 0.0,
																							0.0,            0.0,          0.0,          0.0,          0.0,          inertiaSteer}).transpose();
																		
eeros::math::Matrix<6,1,double> ControlSystemWithJacobi::torqueLim = eeros::math::Matrix<6,1,double>({torqueSaturation,
																							torqueSaturation,
																							torqueSaturation,
																							torqueSaturation,
																							torqueSaturation,
																							torqueSaturation	});

eeros::math::Matrix<6,1,double> ControlSystemWithJacobi::speedLim = eeros::math::Matrix<6,1,double>({ speedLimWheel,//20.0,
																							speedLimWheel,//20.0,
																							speedLimWheel,//20.0,
																							speedLimSteer,//10.36,
																							speedLimSteer,//10.36,
																							speedLimSteer    });//10.36	});
																							
eeros::math::Matrix<6,1,double> ControlSystemWithJacobi::resetValue6 = eeros::math::Matrix<6,1,double>({ 0.0,
																							  0.0,
																							  0.0,
																							  0.0,
																							  0.0,
																							  0.0  });

eeros::math::Matrix<3,1,double> ControlSystemWithJacobi::resetValue3 = eeros::math::Matrix<3,1,double>({ 0.0,
																							   0.0,
																							   0.0  });

eeros::math::Matrix<2,1,double> ControlSystemWithJacobi::resetValue2 = eeros::math::Matrix<2,1,double>({ 0.0,
																							   0.0  });

ControlSystemWithJacobi::ControlSystemWithJacobi(double ts) : 
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

homingspeedsoll({0.0, 0.0, 0.0, 1.0, 1.0, 1.0}), // geschwindigkeitsvorgabe 1[rad]
swHomingOn(1),
speedSaturation(0.0-speedLim,speedLim),

gainDeltaQToQpoint(1.0/ts),
gainKp(kp),
gainKd(kd),

gainInertiaRobot(inertia),

offsetEnc({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), //offsetEnc({0.0, 0.0, 0.0, 1.776, 2.833, 5.6997}), 			// gemmesene Offset Werte wenn Motoren von oben im gegenurzeigersinn Drehen [rad]

ishomed(false),
isReady(false),
checkstartdrive(false),

timedomain("Main time domain",ts,true), 		// TODO make realtime
motModel(gear, invGear, torqueLim),
voltageToPwm(bridgeVoltage),
counter(100), // 100 ms
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
	integratorQpointToQ.getIn().connect(gainDeltaQToQpoint.getOut());
	
	sumOffset.getIn(0).connect(offsetEnc.getOut());
	sumOffset.getIn(1).connect(integratorQpointToQ.getOut());
	
	deMuxActualPos.getIn().connect(sumOffset.getOut());
	muxActualSteerPos.getIn(0).connect(deMuxActualPos.getOut(3));
	muxActualSteerPos.getIn(1).connect(deMuxActualPos.getOut(4));
	muxActualSteerPos.getIn(2).connect(deMuxActualPos.getOut(5));
	
	demuxJoystick.getIn().connect(joystick.getOutVGdes());
	
	muxVeloGlobalxyPhi.getIn(0).connect(demuxJoystick.getOut(0));
	muxVeloGlobalxyPhi.getIn(1).connect(demuxJoystick.getOut(1));
	muxVeloGlobalxyPhi.getIn(2).connect(demuxJoystick.getOut(3));
	
	integratorVGlobalToPosGlobal.getIn().connect(muxVeloGlobalxyPhi.getOut());

	demuxPosGlobal.getIn().connect(integratorVGlobalToPosGlobal.getOut());
	
	jacobi.getInActualPosQ().connect(sumOffset.getOut());
	jacobi.getInOmega().connect(gainDeltaQToQpoint.getOut());
	
	localToGlobal.getInPhiGL().connect(demuxPosGlobal.getOut(2)); // soll phi_Gl wird verwended
	localToGlobal.getInVL().connect(jacobi.getOutVL());
	
	joystick.getInVGactual().connect(localToGlobal.getOutVG());
	
	integratorOdoPointToOdo.getIn().connect(localToGlobal.getOutVG());
	
	sum1.negateInput(1);
	sum1.getIn(0).connect(integratorVGlobalToPosGlobal.getOut());
	sum1.getIn(1).connect(integratorOdoPointToOdo.getOut());
	
	gainKp.getIn().connect(sum1.getOut());
	
	sum2.getIn(0).connect(muxVeloGlobalxyPhi.getOut());
	sum2.getIn(1).connect(gainKp.getOut());
	
	invjacobi.getInPhiGlobal().connect(demuxPosGlobal.getOut(2));
	invjacobi.getInPhiSteer().connect(muxActualSteerPos.getOut());
	invjacobi.getInVglobal().connect(sum2.getOut());

	swHomingOn.getIn(0).connect(invjacobi.getOutOmega());
	swHomingOn.getIn(1).connect(homingspeedsoll.getOut());
	
	speedSaturation.getIn().connect(swHomingOn.getOut());
	
	sum3.negateInput(1);
	sum3.getIn(0).connect(speedSaturation.getOut());
	sum3.getIn(1).connect(gainDeltaQToQpoint.getOut());
	
	gainKd.getIn().connect(sum3.getOut());
	
	gainInertiaRobot.getIn().connect(gainKd.getOut());
	
	homingBlock.getInTorque().connect(gainInertiaRobot.getOut());
	
	motModel.getInTorque().connect(homingBlock.getOutHomingTorque());
	motModel.getInSpeed().connect(gainDeltaQToQpoint.getOut());
	
	voltageToPwm.getInVoltage().connect(motModel.getOutVoltage());
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
	timedomain.addBlock(&muxEncIn);
	timedomain.addBlock(&gainDeltaQToQpoint);
	timedomain.addBlock(&integratorQpointToQ);
	timedomain.addBlock(&offsetEnc);
	timedomain.addBlock(&sumOffset);
	timedomain.addBlock(&deMuxActualPos);
	timedomain.addBlock(&muxActualSteerPos);
	timedomain.addBlock(&joystick);
	timedomain.addBlock(&demuxJoystick);
	timedomain.addBlock(&muxVeloGlobalxyPhi);
	timedomain.addBlock(&integratorVGlobalToPosGlobal);
	timedomain.addBlock(&demuxPosGlobal);
	timedomain.addBlock(&jacobi);
	timedomain.addBlock(&localToGlobal);
	timedomain.addBlock(&integratorOdoPointToOdo);
	timedomain.addBlock(&sum1);
	timedomain.addBlock(&gainKp);
	timedomain.addBlock(&sum2);
	timedomain.addBlock(&invjacobi);
	timedomain.addBlock(&homingspeedsoll);
	timedomain.addBlock(&swHomingOn);
	timedomain.addBlock(&speedSaturation);
	timedomain.addBlock(&sum3);
	timedomain.addBlock(&gainKd);
	timedomain.addBlock(&gainInertiaRobot);
	timedomain.addBlock(&homingBlock);
	timedomain.addBlock(&motModel);
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

ControlSystemWithJacobi::~ControlSystemWithJacobi()
{
	stop();
}


void ControlSystemWithJacobi::start() {
	timedomain.start();
}

void ControlSystemWithJacobi::stop() {
	timedomain.stop();
// 	timedomain.join();
}



void ControlSystemWithJacobi::initReadyToDrive() {				// Offset im Safty oder Sequenzer
	checkStartDrive();

	if(!checkstartdrive){
		std::cout << " check for activated drive is bad! " << std::endl;
		isReady = false;
		integratorVGlobalToPosGlobal.disable();	
		integratorOdoPointToOdo.disable();
	}
	else {
		// Homing Switch
		homingBlock.swSteer1.switchToInput(0);
		homingBlock.swSteer2.switchToInput(0);
		homingBlock.swSteer3.switchToInput(0);
		homingBlock.swWeel1.switchToInput(0);
		homingBlock.swWeel2.switchToInput(0);
		homingBlock.swWeel3.switchToInput(0);

		isReady = true;
	}
}



void ControlSystemWithJacobi::checkStartDrive(){
	integratorVGlobalToPosGlobal.setInitCondition(resetValue3);
	integratorOdoPointToOdo.setInitCondition(resetValue3);
	usleep(50);
	
	integratorVGlobalToPosGlobal.enable();	
	integratorOdoPointToOdo.enable();
	homingspeedsoll.setValue({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
	swHomingOn.switchToInput(1); // 1: inv_Jacobi noch nicht eingeschaltet (Soll speed ist immernoch Homingspeed) 0: inv Jacobi eingeschaltet
	
	usleep(50);
	Vector3 posJoystick;
	Vector3 posJacobi;
	
	posJoystick = integratorVGlobalToPosGlobal.getOut().getSignal().getValue();
	posJacobi = integratorOdoPointToOdo.getOut().getSignal().getValue();

	Vector3 diff;
	diff = posJoystick - posJacobi;
	
	double d = 0.0;

	// Check difference
	for( int i = 0; i < 3; i++){
		if(diff(i) < 0){
			d += -diff(i);			
		}
		else {
			d += diff(i);
		}
	}
	
	if(d > 0.001){
		
		checkstartdrive = false;
	}
	else {
		checkstartdrive = true;
	}
}



void ControlSystemWithJacobi::setHomingToTrue(){
	ishomed = true;
}


bool ControlSystemWithJacobi::isaxisHomed(){
	return ishomed;
}


bool ControlSystemWithJacobi::isreadyToDrive(){
	return isReady;
}



void ControlSystemWithJacobi::initHoming() {
	if(ishomed) return;
	
	swHomingOn.switchToInput(1); // inv_Jacobi  nicht eingeschaltet (Soll speed ist immernoch Homingspeed)
	
	allAxisStopped();
	
	integratorQpointToQ.setInitCondition(resetValue6);                       // Encoder integrator reset
	integratorQpointToQ.enable();
	
};



void ControlSystemWithJacobi::allAxisStopped()
{
	homingBlock.swSteer1.switchToInput(1);
	homingBlock.swSteer2.switchToInput(1);
	homingBlock.swSteer3.switchToInput(1);
	homingBlock.swWeel1.switchToInput(1);
	homingBlock.swWeel2.switchToInput(1);
	homingBlock.swWeel3.switchToInput(1);

	integratorVGlobalToPosGlobal.disable();
	integratorOdoPointToOdo.disable();
}



void ControlSystemWithJacobi::setWheelToHoming(int steer)
{

	if(steer == steer1){	// Homing Rad1
		homingBlock.swSteer1.switchToInput(0); // Rad1 steer get torque 
	}
	else if(steer == steer2){ // Homing Rad2
		homingBlock.swSteer2.switchToInput(0);
	}
	else{ // Homing Rad3
		homingBlock.swSteer3.switchToInput(0);
	}
}
