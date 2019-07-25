#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSGTOSAPOSTROPHE_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSGTOSAPOSTROPHE_HPP

#include <omnimobot/constants.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

namespace omnimobot
{
	class TransGtoSapostrophe : public eeros::control::Block
	{
	public:
		TransGtoSapostrophe();
		virtual ~TransGtoSapostrophe();
		
		virtual eeros::control::Input<eeros::math::Vector<3>>& getInVG() {
			return inV_G;
		}
		
		virtual eeros::control::Input<>& getInPhiGL() {
			return inPhiGL;
		}
		
		virtual eeros::control::Output<DesiredVeloVector>& getOutVSapostrophe() {
			return outV_Sapo;
		}
		
		virtual void run();
		
		protected:
		eeros::control::Input<eeros::math::Vector<3>> inV_G;
		eeros::control::Input<> inPhiGL;
		
		eeros::control::Output<DesiredVeloVector> outV_Sapo;
		
	private:
		double phi;
		eeros::math::Vector<3> v_G;
		eeros::math::Vector<3> veloSap;
		eeros::math::Matrix<3,3> RotGtoSap;
	};
}


#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSGTOSAPOSTROPHE_HPP */