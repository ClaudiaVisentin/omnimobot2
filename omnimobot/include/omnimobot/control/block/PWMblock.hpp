#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_PWMBLOCK_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_PWMBLOCK_HPP

#include <omnimobot/types.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/hal/ScalableOutput.hpp>
#include <eeros/hal/HAL.hpp>
#include <cmath>

// This Block is not used!!

namespace omnimobot
{

	class PWMblock : public eeros::control::Block
	{
	public:
		PWMblock();
		virtual ~PWMblock();
		
		virtual eeros::control::Input<Vector6>& getInVoltage() {
			return inVoltage;
		}
		
		virtual double getOutPwmA(int i) { 	
			return pwmA[i]->get();
		}
		
		virtual double getOutPwmB(int i) { 	
			return pwmB[i]-> get();
		}
		
		virtual void run();
		
	protected:
		eeros::control::Input<Vector6> inVoltage;
		
// 		eeros::hal::PeripheralOutput<double>* pwmA[6];
// 		eeros::hal::PeripheralOutput<double>* pwmB[6];
		eeros::hal::ScalableOutput<double>* pwmA[6]; // TODO check
		eeros::hal::ScalableOutput<double>* pwmB[6];
		
	};
	
}


#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_PWMBLOCK_HPP */