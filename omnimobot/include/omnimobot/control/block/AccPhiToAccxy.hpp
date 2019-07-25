#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_ACCPHITOACCXY_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_ACCPHITOACCXY_HPP

#include <eeros/control/Block1i1o.hpp>
#include <eeros/core/System.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

#include <cmath>

namespace omnimobot 
{

	class AccPhiToAccxy : public eeros::control::Block
	{
	public:
		AccPhiToAccxy();
		virtual ~AccPhiToAccxy();
		
		virtual eeros::control::Input<eeros::math::Vector2>& getInAccPhi() {
			return inAccPhi;
		}
		
		virtual eeros::control::Input<eeros::math::Vector2>& getInPhixy() {
			return inPhixy;
		}
		
		virtual eeros::control::Output<eeros::math::Vector2>& getOutAccxy() {
			return outAccxy;
		}
		
		virtual void run();
		
	protected:
		eeros::control::Input<eeros::math::Vector2> inAccPhi;
		eeros::control::Input<eeros::math::Vector2> inPhixy;
		eeros::control::Output<eeros::math::Vector2> outAccxy;
		
	private:
		
		eeros::math::Vector<2> phiHall;
		eeros::math::Vector<2> AccPhi;
		eeros::math::Vector<2> Accxy;
		double posSchwerpunkt;
		double traegheitStab;
		double masseStab;
		double variableR; // radius r (see doku)
	};
};

#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_ACCPHITOACCXY_HPP */