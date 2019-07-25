#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_HALLSENSORINPUT_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_HALLSENSORINPUT_HPP

#include <omnimobot/types.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Constant.hpp>
#include <omnimobot/control/block/FilterHall.hpp>

namespace omnimobot
{

	class HallSensorInput : public eeros::control::Block
	{
	public:
		HallSensorInput();
		virtual ~HallSensorInput();
		
		
		virtual eeros::control::Output<eeros::math::Vector<2>>& getOutPhixy();
		virtual bool isStickConnected();

		virtual void run();

		
		omnimobot::FilterHall filterHall;
		
	private:

		eeros::control::PeripheralInput<double> hall0;
		eeros::control::PeripheralInput<double> hall1;
		eeros::control::PeripheralInput<double> hall2;
		eeros::control::PeripheralInput<double> hall3;
		
		eeros::control::Constant<eeros::math::Vector<2>> calibration;
		
		eeros::control::Mux<4,double> muxHallValues;
		
		eeros::control::Gain<eeros::math::Vector<2>, double> gainWithFactor;
		eeros::control::Mux<2,double> muxValueOfSums;
		eeros::control::Sum<2,double> sumHall0AndHall2;
		eeros::control::Sum<2,double> sumHall3AndHall1;
		eeros::control::Sum<2,eeros::math::Vector<2>> sumcalibration;

	};
}




#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_HALLSENSORINPUT_HPP */