#include <omnimobot/control/block/PWMblock.hpp>

using namespace omnimobot;
using namespace eeros::math;
using namespace eeros::hal;



PWMblock::PWMblock() :
pwmA{
	HAL::instance().getScalableOutput("pwmAaxis0"),
	HAL::instance().getScalableOutput("pwmAaxis1"),
	HAL::instance().getScalableOutput("pwmAaxis2"),
	HAL::instance().getScalableOutput("pwmAaxis3"),
	HAL::instance().getScalableOutput("pwmAaxis4"),
	HAL::instance().getScalableOutput("pwmAaxis5")},
pwmB{
	HAL::instance().getScalableOutput("pwmBaxis0"),
	HAL::instance().getScalableOutput("pwmBaxis1"),
	HAL::instance().getScalableOutput("pwmBaxis2"),
	HAL::instance().getScalableOutput("pwmBaxis3"),
	HAL::instance().getScalableOutput("pwmBaxis4"),
	HAL::instance().getScalableOutput("pwmBaxis5")}
{
	
}

PWMblock::~PWMblock() { }


void PWMblock::run()
{
	Vector6 dutyCycle = inVoltage.getSignal().getValue() / 24.0; 
	
	for(int i = 0; i < 6; i++) {
		if (dutyCycle(i) > 0.5) dutyCycle(i) = 0.5;
		if (dutyCycle(i) < -0.5) dutyCycle(i) = -0.5;
// 		if (dutyCycle(i) > 0.95) dutyCycle(i) = 0.95;
// 		if (dutyCycle(i) < -0.95) dutyCycle(i) = -0.95;
		
		if (dutyCycle(i) > 0) {
			pwmA[i]->set(dutyCycle(i));
			pwmB[i]->set(0);
		}
		else {
			pwmA[i]->set(0);
			pwmB[i]->set(-dutyCycle(i));
		}
	}
}
