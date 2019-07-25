#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_JACOBIAN_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_JACOBIAN_HPP

#include <omnimobot/types.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <omnimobot/Jacobi.hpp>

namespace omnimobot
{

	class Jacobian : public eeros::control::Block
	{
	public:
		Jacobian();
		virtual ~Jacobian();
		
		virtual eeros::control::Input<Vector6>& getInOmega() {
			return inOmega;
		}
		
		virtual eeros::control::Input<eeros::math::Vector<6>>& getInActualPosQ() {
			return inActualPosQ;
		}

		virtual eeros::control::Output<eeros::math::Vector3>& getOutVL() {
			return outV_L;
		}
		
		virtual eeros::control::Output<>& getOutPhiGLd() {
			return outPhiGLd;
		}
		
		virtual double getSlip() {
			return jacobi.getNumberOfSlips();
		}
			
		virtual void run();

		
	protected:
		eeros::control::Input<Vector6> inOmega;
		eeros::control::Input<eeros::math::Vector<6>> inActualPosQ;
		eeros::control::Output<> outPhiGLd;
		eeros::control::Output<eeros::math::Vector3> outV_L;

	private:
		eeros::math::Vector<6> omega_w;
		eeros::math::Vector<3> v_L;
		eeros::math::Vector<3> v_L_Out;
		eeros::math::Vector<3> phi_j;

		Jacobi jacobi;

		
	};
	
}
#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_JACOBIAN_HPP */
