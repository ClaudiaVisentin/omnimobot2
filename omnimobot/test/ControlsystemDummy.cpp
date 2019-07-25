#include "ControlsystemDummy.hpp"
#include <omnimobot/safety/OmniSafetyProperties.hpp>
#include <math.h>
#include <unistd.h>
#include <iostream>

using namespace eeros::control;
using namespace omnimobot;
using namespace eeros::math;


eeros::math::Matrix<6,6,double> ControlsystemDummy::gear = eeros::math::Matrix<6,6,double>({	18.0, 0.0,  0.0, 0.0,    0.0,    0.0, // (18) gear Antrieb
																						0.0, 18.0,  0.0, 0.0,    0.0,    0.0,
																						0.0,  0.0, 18.0, 0.0,    0.0,    0.0,
																						0.0,  0.0,  0.0, 36.75,  0.0,    0.0, // (36.75) gear Lenkung
																						0.0,  0.0,  0.0, 0.0,    36.75,  0.0,
																						0.0,  0.0,  0.0, 0.0,    0.0,    36.75}).transpose(); // transponiert, da die Matrix 1.Spalte dann 2. Spalte fühlt

eeros::math::Matrix<6,6,double> ControlsystemDummy::invGear = eeros::math::Matrix<6,6,double>({	1.0/18.0, 0.0,      0.0,      0.0,       0.0,      0.0,
																							0.0,     1.0/18.0, 0.0,      0.0,       0.0,      0.0,
																							0.0,     0.0,      1.0/18.0, 0.0,       0.0,      0.0,
																							0.0,     0.0,      0.0,      1.0/36.75, 0.0,      0.0,
																							0.0,     0.0,      0.0,      0.0,       1.0/36.75,0.0,
																							0.0,     0.0,      0.0,      0.0,       0.0,      1.0/36.75}).transpose();

eeros::math::Matrix<6,6,double> ControlsystemDummy::inertia = eeros::math::Matrix<6,6,double>({	0.034279,   0.0,       0.0,       0.0,       0.0,       0.0, // (0.034279) Trägheit Antrieb
																							0.0,       0.034279,   0.0,       0.0,       0.0,       0.0,
																							0.0,       0.0,       0.034279,   0.0,       0.0,       0.0,
																							0.0,       0.0,       0.0,       0.0668063,   0.0,       0.0, // (0.0668063) Trägheit Lankung
																							0.0,       0.0,       0.0,       0.0,       0.0668063,   0.0,
																							0.0,       0.0,       0.0,       0.0,       0.0,       0.0668063}).transpose();
																							
eeros::math::Matrix<6,1,double> ControlsystemDummy::torqueLim = eeros::math::Matrix<6,1,double>({1.0,
																							1.0,
																							1.0,
																							1.0,
																							1.0,
																							1.0	});

eeros::math::Matrix<6,1,double> ControlsystemDummy::speedLim = eeros::math::Matrix<6,1,double>({ 20.0,
																							20.0,
																							20.0,
																							10.36,
																							10.36,
																							10.36	});


ControlsystemDummy::ControlsystemDummy(double ts) : 
//	joystick("/dev/input/js0"),
	joystickConst({0.0, 0.0, 0.0, 0.0}),//anpassen
	enc0("q0"),
	enc1("q1"),
	enc2("q2"),
	enc3("q3"),
	enc4("q4"),
	enc5("q5"),
// 	pwmA0("pwmAaxis0"),
// 	pwmA1("pwmAaxis1"),
// 	pwmA2("pwmAaxis2"),
// 	pwmA3("pwmAaxis3"),
// 	pwmA4("pwmAaxis4"),
// 	pwmA5("pwmAaxis5"),
// 	pwmB0("pwmBaxis0"),
// 	pwmB1("pwmBaxis1"),
// 	pwmB2("pwmBaxis2"),
// 	pwmB3("pwmBaxis3"),
// 	pwmB4("pwmBaxis4"),
// 	pwmB5("pwmBaxis5"),

	
	offsetEnc({0.0, 0.0, 0.0, 1.776, 2.833, 5.6997}), 			// gemmesene Offset Werte wenn Motoren von oben im gegenurzeigersinn Drehen [rad]
	
	homed(false),
	
	timedomain("Main time domain",ts,false) 		// TODO make realtime
	
// 	robotControlBlock( km,  R,  gear, invGear, inertia,  kv,  kp, torqueLim, speedLim),
// 	voltageToPwm(bridgeVoltage)
	{
		muxEncPosIn.getIn(0).connect(enc0.getOut());
		muxEncPosIn.getIn(1).connect(enc1.getOut());
		muxEncPosIn.getIn(2).connect(enc2.getOut());
		muxEncPosIn.getIn(3).connect(enc3.getOut());
		muxEncPosIn.getIn(4).connect(enc4.getOut());
		muxEncPosIn.getIn(5).connect(enc5.getOut());
		
		sumOffset.getIn(0).connect(offsetEnc.getOut());
		sumOffset.getIn(1).connect(muxEncPosIn.getOut());
		
// 		robotControlBlock.getPosIn().connect(sumOffset.getOut());
// 		diffEncPos.getIn().connect(sumOffset.getOut());
// 		robotControlBlock.getSpeedIn().connect(diffEncPos.getOut());
// 		demuxJoystick.getIn().connect(joystickConst.getOut());// anpassen
// 		muxVeloLocalIn.getIn(0).connect(demuxJoystick.getOut(0));
// 		muxVeloLocalIn.getIn(1).connect(demuxJoystick.getOut(1));
// 		muxVeloLocalIn.getIn(2).connect(demuxJoystick.getOut(3));
		
// 		robotControlBlock.getVeloLocalIn().connect(muxVeloLocalIn.getOut());
//  		voltageToPwm.getInVoltage().connect(robotControlBlock.getVoltageOut());
// 		deMuxPwmAOut.getIn().connect(voltageToPwm.getOutPwmA());
// 		deMuxPwmBOut.getIn().connect(voltageToPwm.getOutPwmB());
		
// 		pwmA0.getIn().connect(deMuxPwmAOut.getOut(0));
// 		pwmA1.getIn().connect(deMuxPwmAOut.getOut(1));
// 		pwmA2.getIn().connect(deMuxPwmAOut.getOut(2));
// 		pwmA3.getIn().connect(deMuxPwmAOut.getOut(3));
// 		pwmA4.getIn().connect(deMuxPwmAOut.getOut(4));
// 		pwmA5.getIn().connect(deMuxPwmAOut.getOut(5));
// 		pwmB0.getIn().connect(deMuxPwmBOut.getOut(0));
// 		pwmB1.getIn().connect(deMuxPwmBOut.getOut(1));
// 		pwmB2.getIn().connect(deMuxPwmBOut.getOut(2));
// 		pwmB3.getIn().connect(deMuxPwmBOut.getOut(3));
// 		pwmB4.getIn().connect(deMuxPwmBOut.getOut(4));
// 		pwmB5.getIn().connect(deMuxPwmBOut.getOut(5));
		
		
		
		timedomain.addBlock(&enc0);
		timedomain.addBlock(&enc1);
		timedomain.addBlock(&enc2);
		timedomain.addBlock(&enc3);
		timedomain.addBlock(&enc4);
		timedomain.addBlock(&enc5);
		timedomain.addBlock(&offsetEnc);
		timedomain.addBlock(&muxEncPosIn);
		timedomain.addBlock(&sumOffset);
		timedomain.addBlock(&diffEncPos);
		timedomain.addBlock(&joystickConst);// anpassen
		timedomain.addBlock(&demuxJoystick);
 		timedomain.addBlock(&muxVeloLocalIn);
// 		timedomain.addBlock(&robotControlBlock);
// 		timedomain.addBlock(&voltageToPwm);
// 		timedomain.addBlock(&deMuxPwmAOut);
// 		timedomain.addBlock(&deMuxPwmBOut);
// 		timedomain.addBlock(&pwmA0);
// 		timedomain.addBlock(&pwmA1);
// 		timedomain.addBlock(&pwmA2);
// 		timedomain.addBlock(&pwmA3);
// 		timedomain.addBlock(&pwmA4);
// 		timedomain.addBlock(&pwmA5);
// 		timedomain.addBlock(&pwmB0);
// 		timedomain.addBlock(&pwmB1);
// 		timedomain.addBlock(&pwmB2);
// 		timedomain.addBlock(&pwmB3);
// 		timedomain.addBlock(&pwmB4);
// 		timedomain.addBlock(&pwmB5);
		
//		std::cout << "CS Thread ID: " << timedomain.getId()<< std::endl;
		
}


void ControlsystemDummy::start() {
	timedomain.start();
}

void ControlsystemDummy::stop() {
	timedomain.stop();
	timedomain.join();
}

bool ControlsystemDummy::homeingFinished() {				// Offset im Safty oder Sequenzer
	if(homed ) return false;						
// 	robotControlBlock.setHomingSpeed({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
// 	robotControlBlock.swHomingOn.switchToInput(1); // inv_Jacobi noch nicht eingeschaltet (Soll speed ist immernoch Homingspeed)
// 	
// 	// Homing Switch
// 	robotControlBlock.homingBlock.swSteer1.switchToInput(0);
// 	robotControlBlock.homingBlock.swSteer2.switchToInput(0);
// 	robotControlBlock.homingBlock.swSteer3.switchToInput(0);
// 	robotControlBlock.homingBlock.swWeel1.switchToInput(0);
// 	robotControlBlock.homingBlock.swWeel2.switchToInput(0);
// 	robotControlBlock.homingBlock.swWeel3.switchToInput(0);

	homed = true;
	return true;
}

bool ControlsystemDummy::axisHomed(){
	return homed;
}

void ControlsystemDummy::controlOn() {
	//robotControlBlock.swHomingOn.switchToInput(0);
}

bool ControlsystemDummy::readyToHoming() {
	if(homed) return false;
/*	
	robotControlBlock.swHomingOn.switchToInput(1); // inv_Jacobi  nicht eingeschaltet (Soll speed ist immernoch Homingspeed)
	
	allAxisStopped();
	
	robotControlBlock.integrator.setInitCondition(robotControlBlock.swHomingOn.getOut().getSignal().getValue());		// Integrator reset
	*/
	return true;
	
};



void ControlsystemDummy::allAxisStopped()
{
// 	robotControlBlock.homingBlock.swSteer1.switchToInput(1);
// 	robotControlBlock.homingBlock.swSteer2.switchToInput(1);
// 	robotControlBlock.homingBlock.swSteer3.switchToInput(1);
// 	robotControlBlock.homingBlock.swWeel1.switchToInput(1);
// 	robotControlBlock.homingBlock.swWeel2.switchToInput(1);
// 	robotControlBlock.homingBlock.swWeel3.switchToInput(1);

}
