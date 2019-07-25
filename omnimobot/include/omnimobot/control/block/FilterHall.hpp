#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_FILTERHALL_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_FILTERHALL_HPP

#include <omnimobot/types.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>


namespace omnimobot
{

	class FilterHall : public eeros::control::Block
	{
	public:
		FilterHall();
		virtual ~FilterHall();
		
		virtual eeros::control::Input<eeros::math::Vector<4>>& getInHall() {
			return inHall;
		}
		
		virtual eeros::control::Output<double>& getOutHall0() {
			return outHall0;
		}
		
		virtual eeros::control::Output<double>& getOutHall1() {
			return outHall1;
		}
		
		virtual eeros::control::Output<double>& getOutHall2() {
			return outHall2;
		}
		
		virtual eeros::control::Output<double>& getOutHall3() {
			return outHall3;
		}
		
		virtual bool isStickConnected() {
			return stickConnected;
		}
		
		virtual void run();

		
	private:
		eeros::control::Input<eeros::math::Vector<4>> inHall;
		eeros::control::Output<double> outHall0;
		eeros::control::Output<double> outHall1;
		eeros::control::Output<double> outHall2;
		eeros::control::Output<double> outHall3;
		
		eeros::math::Vector<4> outHallValue;
		eeros::math::Vector<4> inHallValue;
		eeros::math::Vector<4> hallValuePrev;
		
		bool stickConnected;
		
	};
	
}


#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_FILTERHALL_HPP */