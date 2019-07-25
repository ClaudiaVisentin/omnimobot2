#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_PHITOTIP_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_PHITOTIP_HPP


#include <eeros/control/Block1i1o.hpp>
#include <eeros/core/System.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

#include <cmath>

namespace omnimobot 
{

	class PhiToTip : public eeros::control::Block
	{
		
	public:
		PhiToTip();
		virtual ~PhiToTip();
		
		virtual eeros::control::Input<eeros::math::Vector3>& getInOdometry() {
			return inPosRobot;
		}
		
		virtual eeros::control::Input<eeros::math::Vector2>& getInPhixy() {
			return inPhixy;
		}
		
		virtual eeros::control::Output<eeros::math::Vector2>& getOutTip() {
			return outTip;
		}
		
		virtual void run();


	protected:
		eeros::control::Input<eeros::math::Vector3> inPosRobot;
		eeros::control::Input<eeros::math::Vector2> inPhixy;
		eeros::control::Output<eeros::math::Vector2> outTip;
		
	private:	
		double stablaenge;
		eeros::math::Vector<2> phiHall;
		eeros::math::Vector<3> posRobot;
		eeros::math::Vector<2> XYtip;
		
	};
};

#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_PHITOTIP_HPP */