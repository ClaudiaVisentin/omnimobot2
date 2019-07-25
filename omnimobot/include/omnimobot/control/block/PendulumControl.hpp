#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_PENDULUMCONTROL_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_PENDULUMCONTROL_HPP

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/D.hpp>
#include <eeros/math/Matrix.hpp>
#include <omnimobot/types.hpp>

#include <omnimobot/control/block/AccPhiToAccxy.hpp>
#include <omnimobot/control/block/AccTipToPhi.hpp>
#include <omnimobot/control/block/PhiToTip.hpp>
#include <omnimobot/control/block/I.hpp>
#include <omnimobot/control/block/MeasuringDataBlock2.hpp> 
#include <omnimobot/control/block/MeasuringDataBlock3.hpp> 

namespace omnimobot {
	
	class PendulumControl: public eeros::control::Block {
		
	public:
		PendulumControl(); 
		
		virtual eeros::control::Input<eeros::math::Vector2>& getInPosTip();
		
		virtual eeros::control::Input<eeros::math::Vector2>& getInPhixy();
		
		virtual eeros::control::Input<eeros::math::Vector3>& getInOdometry();
		
		virtual eeros::control::Output<eeros::math::Vector2>& getOutVelo();
		
		virtual void run();
		
		omnimobot::I<eeros::math::Vector2> integrator;
		
		PhiToTip phi_To_Tip;
		AccPhiToAccxy aPhi_To_axy;
		AccTipToPhi aTip_To_Phi;
		
		eeros::control::Sum<2,eeros::math::Vector2> SumTip;
		eeros::control::Sum<2,eeros::math::Vector2> SumPhi;
		
		MeasuringDataBlock2 measuringTipDes; // only for test
		MeasuringDataBlock2 measuringTipActual; // only for test
		MeasuringDataBlock3 measuringPosActual; // only for test
		
	private:

		eeros::control::Gain<eeros::math::Vector2,double> phixyIn;
		eeros::control::Gain<eeros::math::Vector2,double> gKpPos;
		eeros::control::Gain<eeros::math::Vector2,double> gKdPos;
		eeros::control::Gain<eeros::math::Vector2,double> gKpPhi;
		eeros::control::Gain<eeros::math::Vector2,double> gKdPhi;
		eeros::control::Gain<eeros::math::Vector3,double> inActual;
		eeros::control::Gain<eeros::math::Vector2,double> inDesired;
		
		eeros::control::D<eeros::math::Vector2> diffTip;
		eeros::control::D<eeros::math::Vector2> diffPhi;
		
		
		eeros::control::Sum<2,eeros::math::Vector2> SumPDTip;

		eeros::control::Sum<2,eeros::math::Vector2> SumPDPhi;

	};
};



#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_PENDULUMCONTROL_HPP */