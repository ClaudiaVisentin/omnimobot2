#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_ROBOTCONTROL_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_ROBOTCONTROL_HPP

#include <omnimobot/control/block/Homingblock.hpp>
#include <omnimobot/control/block/MotorModel.hpp>
#include <omnimobot/control/block/I.hpp>
#include <omnimobot/control/block/InvJacobian.hpp>

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Switch.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/math/Matrix.hpp>
#include <omnimobot/types.hpp>


namespace omnimobot {
	
	class RobotControl : public eeros::control::Block {
		
	public:
		RobotControl( eeros::math::Matrix<6,6> gear, eeros::math::Matrix<6,6> invgear, eeros::math::Matrix<6,6> inertia, Vector6 torqueLim, Vector6 speedLim);
		virtual ~RobotControl();
		
		virtual eeros::control::Input<eeros::math::Vector3>& getInVeloGlobal();
		virtual eeros::control::Input<>& getInGlobalPhi();
		virtual eeros::control::Input<Vector6>& getInSpeedActualQ();
		virtual eeros::control::Input<Vector6>& getInPosActualQ();
		virtual eeros::control::Output<Vector6>& getOutVoltage();
		
		virtual void run();
		
		virtual void setHomingSpeed(Vector6 u);
		
		eeros::control::Switch<2, Vector6> swHomingOn; 
		eeros::control::Switch<2, Vector6> swSafetystopOn; // Input 1 is Safetystop
		eeros::control::Constant<Vector6> homingspeedsoll;
		eeros::control::Saturation<Vector6> speedSaturation; 
		
		HomingBlock homingBlock;
		
		InvJacobian invjacobi;
		
		MotorModel motModel;
		
		I<Vector6> integrator;
	
		eeros::control::Sum<2, Vector6> sum1;
		eeros::control::Sum<2, Vector6> sum2;
		eeros::control::Mux<3, double> posSteerInMux;
		eeros::control::DeMux<6, double> posInDeMux;
		
		eeros::control::Sum<2, Vector6> sum3;
		eeros::control::Gain<Vector6, double> kdGain;
		eeros::control::Gain<Vector6, double> kpGain;
		eeros::control::Gain<Vector6, double> inSpeedActualQ;
		eeros::control::Gain<Vector6, double> inPosActualQ;
		eeros::control::Gain<Vector6, eeros::math::Matrix<6,6>> inertiaRobot;
		
	private:
		eeros::control::Constant<Vector6> safetyStopValue;
	
	};
};

#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_ROBOTCONTROL_HPP */