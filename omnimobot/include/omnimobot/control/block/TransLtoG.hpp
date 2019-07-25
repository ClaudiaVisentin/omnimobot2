#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSLTOG_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSLTOG_HPP

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

namespace omnimobot
{
	class TransLtoG : public eeros::control::Block
	{
	public:
		TransLtoG();
		virtual ~TransLtoG();
		
		virtual eeros::control::Input<eeros::math::Vector<3>>& getInVL() {
			return inV_L;
		}
		
		virtual eeros::control::Input<>& getInPhiGL() {
			return inPhiGL;
		}
		
		virtual eeros::control::Output<eeros::math::Vector<3>>& getOutVG() {
			return outV_G;
		}
		
		virtual void run();

		protected:
		eeros::control::Input<eeros::math::Vector<3>> inV_L;
		eeros::control::Input<> inPhiGL;
		
		eeros::control::Output<eeros::math::Vector<3>> outV_G;
		
	private:
		double phi;
		eeros::math::Vector<3> v_L;
		eeros::math::Vector<3> veloGlobal;
		eeros::math::Matrix<3,3> RotLtoG;
	};
}


#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSLTOG_HPP */