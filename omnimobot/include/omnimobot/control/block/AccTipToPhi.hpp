#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_ACCTIPTOPHI_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_ACCTIPTOPHI_HPP


#include <eeros/control/Block1i1o.hpp>
#include <eeros/core/System.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>


namespace omnimobot 
{
	class AccTipToPhi : public eeros::control::Block
	{
		
	public:
		AccTipToPhi();
		virtual ~AccTipToPhi();
		
		virtual eeros::control::Input<eeros::math::Vector2>& getInAccTip() {
			return inAccTip;
		}
		
		virtual eeros::control::Output<eeros::math::Vector2>& getOutPhiSoll() {
			return outPhiSoll;
		}
		
		virtual void run();
		
	protected:
		eeros::control::Input<eeros::math::Vector2> inAccTip;
		eeros::control::Output<eeros::math::Vector2> outPhiSoll;
		
	private:
		eeros::math::Vector<2> accTip;
		eeros::math::Vector<2> phisoll;
	};
};

#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_ACCTIPTOPHI_HPP */