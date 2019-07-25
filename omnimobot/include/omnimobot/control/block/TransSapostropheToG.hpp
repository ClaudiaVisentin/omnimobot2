#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSSAPOSTROPHETOG_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSSAPOSTROPHETOG_HPP

#include <omnimobot/constants.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

namespace omnimobot
{
	class TransSapostropheToG : public eeros::control::Block
	{
	public:
		TransSapostropheToG();
		virtual ~TransSapostropheToG();
		
		virtual eeros::control::Input<DesiredVeloVector>& getInVSapostrophe() {
			return inV_Sapo;
		}
		
		virtual eeros::control::Input<>& getInPhiGL() {
			return inPhiGL;
		}
		
		virtual eeros::control::Output<eeros::math::Vector<3>>& getOutVG() {
			return outV_G;
		}
		
		virtual void run();

		protected:
		eeros::control::Input<DesiredVeloVector> inV_Sapo;
		eeros::control::Input<> inPhiGL;
		
		eeros::control::Output<eeros::math::Vector<3>> outV_G;
		
	private:
		double phi;
		eeros::math::Vector<3> v_G;
		eeros::math::Vector<3> veloSap;
		eeros::math::Matrix<3,3> RotSaptoG;
	};
}


#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSSAPOSTROPHETOG_HPP */