#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_VOLTAGETOPWM_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_VOLTAGETOPWM_HPP

#include <omnimobot/types.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <cmath>

namespace omnimobot
{

	class VoltageToPWM : public eeros::control::Block
	{
	public:
		VoltageToPWM(double bridgeVoltage);
		virtual ~VoltageToPWM();
		
		virtual eeros::control::Input<Vector6>& getInVoltage() {
			return inVoltage;
		}
		
		virtual eeros::control::Output<Vector6>& getOutPwmA() {
			return outpwmA;
		}
		
		virtual eeros::control::Output<Vector6>& getOutPwmB() {
			return outpwmB;
		}
		
		virtual void run();
		
		virtual void setBridgeVoltage(double bV);
		
	protected:
		
		eeros::control::Input<Vector6> inVoltage;
		eeros::control::Output<Vector6> outpwmA;
		eeros::control::Output<Vector6> outpwmB;
		
	private:
		
		double bridgeV;
		eeros::math::Vector<6> dutyCycle;
		eeros::math::Vector<6> pwmATemp;
		eeros::math::Vector<6> pwmBTemp;

	};
	
}




#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_VOLTAGETOPWM_HPP */