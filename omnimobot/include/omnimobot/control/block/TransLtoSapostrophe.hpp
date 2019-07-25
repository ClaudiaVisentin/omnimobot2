#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSLTOSAPOSTROPHE_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSLTOSAPOSTROPHE_HPP


#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

namespace omnimobot
{
	class TransLtoSapostrophe : public eeros::control::Block
	{
	public:
		TransLtoSapostrophe();
		virtual ~TransLtoSapostrophe();
		
		virtual eeros::control::Input<eeros::math::Vector<3>>& getInVL() {
			return inV_L;
		}
		
		virtual eeros::control::Output<eeros::math::Vector<3>>& getOutVSapostrophe() {
			return outV_Sapostrophe;
		}
		
		virtual void run();

		
		protected:
		eeros::control::Input<eeros::math::Vector<3>> inV_L;
		
		eeros::control::Output<eeros::math::Vector<3>> outV_Sapostrophe;
		
	private:
		double phi;
		double cosPhi;
		double sinPhi;
		eeros::math::Vector<3> v_L;
		eeros::math::Vector<3> v_Sap;
		eeros::math::Matrix<3,3> RotLtoSap;
	};
}


#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TRANSLTOSAPOSTROPHE_HPP */