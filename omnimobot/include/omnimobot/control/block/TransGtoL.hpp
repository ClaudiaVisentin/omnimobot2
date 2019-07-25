#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSGTOL_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSGTOL_HPP


#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

namespace omnimobot
{
	class TransGtoL : public eeros::control::Block
	{
	public:
		TransGtoL();
		virtual ~TransGtoL();
		
		virtual eeros::control::Input<eeros::math::Vector<3>>& getInVG() {
			return inV_G;
		}
		
		virtual eeros::control::Input<>& getInPhiGL() {
			return inPhiGL;
		}
		
		virtual eeros::control::Output<eeros::math::Vector<3>>& getOutVL() {
			return outV_L;
		}
		
		virtual void run();

		protected:
		eeros::control::Input<eeros::math::Vector<3>> inV_G;
		eeros::control::Input<> inPhiGL;
		eeros::control::Output<eeros::math::Vector<3>> outV_L;
		
	private:
		double phi;
		eeros::math::Vector<3> v_G;
		eeros::math::Vector<3> veloLokal;
		eeros::math::Matrix<3,3> RotGtoL;
	};
}


#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSGTOL_HPP */