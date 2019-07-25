#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_MOTORMODEL_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_MOTORMODEL_HPP

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/math/Matrix.hpp>
#include <omnimobot/types.hpp>


namespace omnimobot {
	
	class MotorModel : public eeros::control::Block {
		
	public:
		MotorModel( eeros::math::Matrix<6,6> gear, eeros::math::Matrix<6,6> invGear, Vector6 torqueLim);
		virtual ~MotorModel();
		
		virtual eeros::control::Input<Vector6>& getInTorque();
		virtual eeros::control::Input<Vector6>& getInSpeed();
		virtual eeros::control::Output<Vector6>& getOutVoltage();
		
		virtual void run();
		
	private:
		eeros::control::Gain<Vector6, eeros::math::Matrix<6,6>> invGearGain; // <out,gain>
		eeros::control::Gain<Vector6, eeros::math::Matrix<6,6>> gearAndMotConst;
		eeros::control::Gain<Vector6, double> invMotConst;
		eeros::control::Gain<Vector6, double> rotResistor;
		eeros::control::Saturation<Vector6> torqueSaturation;

		eeros::control::Sum<2, Vector6> sum;
		
	};
};
#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_MOTORMODEL_HPP */
