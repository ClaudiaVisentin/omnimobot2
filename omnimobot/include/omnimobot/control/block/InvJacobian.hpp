#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_INVJACOBIAN_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_INVJACOBIAN_HPP

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <omnimobot/Jacobi.hpp>

namespace omnimobot
{

	class InvJacobian : public eeros::control::Block
	{
	public:
		InvJacobian();
		InvJacobian(eeros::math::Vector3 phi_j_0, eeros::math::Vector3 phi_ij_0);
		virtual ~InvJacobian();
		
		virtual eeros::control::Input<eeros::math::Vector3>& getInVglobal() {
			return in_V_global;
		}
		
		virtual eeros::control::Input<eeros::math::Vector3>& getInPhiSteer() {
			return inPhiSteer;
		}
			
		virtual eeros::control::Input<>& getInPhiGlobal() {
			return phi_Global;
		}	
			
		virtual eeros::control::Output<Vector6>& getOutOmega() {
			return outOmega;
		}
		
		virtual eeros::control::Output<eeros::math::Vector3>& getOutOmegaSteer() {
			return outOmegaSteer;
		}
			
		virtual void run();

		
	protected:
		eeros::control::Input<eeros::math::Vector3> in_V_global;
		eeros::control::Input<eeros::math::Vector3> inPhiSteer;
		eeros::control::Input<> phi_Global;
		eeros::control::Output<Vector6> outOmega;
		eeros::control::Output<eeros::math::Vector3> outOmegaSteer;

	private:
		eeros::math::Vector<3> v_GL;
		eeros::math::Vector<6> omega_w;
		eeros::math::Vector<3> phi_ij;
		eeros::math::Vector<6> omega;
		eeros::math::Vector<3> omega_steer;
		double phi_GL;
		
		Jacobi invjacobi;

	};
	
}
#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_INVJACOBIAN_HPP */
