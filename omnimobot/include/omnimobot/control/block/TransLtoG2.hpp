#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSLTOG2_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSLTOG2_HPP


#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

namespace omnimobot
{
	class TransLtoG2 : public eeros::control::Block
	{
	public:
		TransLtoG2();
		virtual ~TransLtoG2();
		
		virtual eeros::control::Input<eeros::math::Vector<2>>& getInLocal() {
			return inLocal;
		}
		
		virtual eeros::control::Input<>& getInPhiGL() {
			return inPhiGL;
		}
		
		virtual eeros::control::Output<eeros::math::Vector<2>>& getOutGlobal() {
			return outGlobal;
		}
		
		virtual void run();

		protected:
		eeros::control::Input<eeros::math::Vector<2>> inLocal;
		eeros::control::Input<> inPhiGL;
		
		eeros::control::Output<eeros::math::Vector<2>> outGlobal;
		
	private:
		double phi;
		eeros::math::Vector<2> inL;
		eeros::math::Vector<2> outG;
		eeros::math::Matrix<2,2> RotLtoG;
	};
}


#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSLTOG2_HPP */