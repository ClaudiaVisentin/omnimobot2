#include <omnimobot/control/block/VoltageToPWM.hpp>

using namespace omnimobot;
using namespace eeros::math;
using namespace std;

VoltageToPWM::VoltageToPWM(double bridgeVoltage):
bridgeV(bridgeVoltage)
{
	dutyCycle.zero();
	pwmATemp.zero();
	pwmBTemp.zero();
}

VoltageToPWM::~VoltageToPWM() { }

// Um den Wert null vorgeben zu können, wird beim maxon pwm: 10% == -15A, 90% == 15A => 40% unterschied 
// dadurch ergitbt sich eine Dutycycle umrechnung von d*0.4 + 0.5 zusätzlich muss für eine Richtungsänderung ein zusätzliches signal gegeben werden


void VoltageToPWM::run()
{
	dutyCycle = inVoltage.getSignal().getValue() / bridgeV;	
	
	
	for(int i = 0; i < 6; i++) {
		if (dutyCycle(i) > 0.95) dutyCycle(i) = 0.95;
		if (dutyCycle(i) < -0.95) dutyCycle(i) = -0.95;
// 		if (dutyCycle(i) > 0.6) dutyCycle(i) = 0.6;
// 		if (dutyCycle(i) < -0.6) dutyCycle(i) = -0.6;
		
		if (dutyCycle(i) > 0) {
			pwmATemp(i) = dutyCycle(i);
			pwmBTemp(i) = 0.0;
		}
		else {
			pwmATemp(i) = 0.0;
			pwmBTemp(i) = -dutyCycle(i);
		}
	}
	outpwmA.getSignal().setValue(pwmATemp);
	outpwmA.getSignal().setTimestamp(inVoltage.getSignal().getTimestamp());
	outpwmB.getSignal().setValue(pwmBTemp);
	outpwmB.getSignal().setTimestamp(inVoltage.getSignal().getTimestamp());
	

}

void VoltageToPWM::setBridgeVoltage(double bV)
{
	bridgeV = bV;
}
