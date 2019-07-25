#ifndef __CH_NTB_OMNIMOBOT_JACOBI_HPP
#define __CH_NTB_OMNIMOBOT_JACOBI_HPP

#include <omnimobot/types.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>

/**********************************************************
 * File:     Jacobi.hpp                              
 * Created:  2014  S.Landis                      
 * Changes:	 none
 * ------------------------------------------------------ 
 * Description:                                           
 * Kinematic of the OmniMoBot                                      
 **********************************************************/

namespace omnimobot
{
	class Jacobi 
	{
	public:
		Jacobi();
		virtual ~Jacobi();

		/** Calculate jacobi Matrix of the OmniMOBot. 
		 * @param omega	joint coordinates omega_weels[rad/s], omega_steers[rad/s], [w_Rad1, w_Rad2, w_Rad3, s_Rad1, s_Rad2, s_Rad3]
		 * @param v_global	[vx_G [m/s], vy_G [m/s], omega_G [rad/s]]
		 */
		virtual void calculateInvJacobiGtoR(Vector6& omega, const eeros::math::Vector3& v_global, const eeros::math::Vector3 phi_LR, const double phi_GL); 
		
		
		
		/** Calculate jacobi Matrix of the OmniMOBot. 
		 * @param omega_Rad	joint coordinates omega_weels[rad/s], omega_steers[rad/s], [w_Rad1, w_Rad2, w_Rad3, s_Rad1, s_Rad2, s_Rad3]
		 * @param v_local	[vx_L [m/s], vy_L [m/s], omega_L [rad/s]]
		 */
		virtual void calculateJacobiRtoL(const Vector6& omega, eeros::math::Vector3& v_local, const eeros::math::Vector3 phi_LR_j); 
		
		
		
		/** Returns values of coordinate system {R}.
		 * @param omega	joint coordinates omega_weels[rad/s], omega_steers[rad/s], [w_Rad1, w_Rad2, w_Rad3, s_Rad1, s_Rad2, s_Rad3]
		 * @param v_global	[vx_G [m/s], vy_G [m/s], omega_G [rad/s]]
		 */
		virtual Vector6 getOmega( Vector6& omega, const eeros::math::Vector3& v_global, const eeros::math::Vector3 phi_LR, const double phi_GL);
		
		

		/** Returns values of coordinate system {L}.
		 * @param omega_Rad	joint coordinates omega_weels[rad/s], omega_steers[rad/s], [w_Rad1, w_Rad2, w_Rad3, s_Rad1, s_Rad2, s_Rad3]
		 * @param v_local	[vx_L [m/s], vy_L [m/s], omega_L [rad/s]]
		 */
		virtual eeros::math::Vector3 getVeloLocal(const Vector6& omega, eeros::math::Vector3& v_local, const eeros::math::Vector3 phi_LR_j);
		
		
		virtual double getNumberOfSlips();
		
		eeros::math::Vector3 phi_LR_ij; // ev. wieder private
		eeros::math::Vector3 phi_LR_j;

		
	private:
		
		eeros::math::Vector<3>  d_theta;
		eeros::math::Vector<4>  omega_4;
		eeros::math::Matrix<3,3>v_GR_L_i;
		eeros::math::Matrix<3,1>l_i_0;
		eeros::math::Matrix<3,1>l_i_1;
		eeros::math::Matrix<3,1>l_i_2;
		eeros::math::Vector<3>v_GRx_R;
		eeros::math::Vector<3>v_GRy_R;
		eeros::math::Vector<3>v_GR_R;
		eeros::math::Matrix<3,3>Rot_RtoL;
// 		eeros::math::Matrix<6,3>jacobi_aug_6x3;  // temp invjacobi
// 		eeros::math::Matrix<4,3>jacobi_aug_4x3;  // temp invjacobi
// 		eeros::math::Matrix<3,6>pseudo_jacobi_LPT_3x6; //pseudo jacobimatrix
// 		eeros::math::Matrix<3,4>pseudo_jacobi_LPT_3x4; //pseudo jacobimatrix
		eeros::math::Matrix<3,3>tmpMatrix; 
		
		
		double v_l_12_abs;
		double v_l_13_abs;
		double v_l_23_abs;
		
		double l_1_abs;
		double l_2_abs;
		double l_3_abs;
		
		double l_12_abs;
		double l_13_abs;
		double l_23_abs;
		double v_projection_12_diff_abs;
		double v_projection_13_diff_abs;
		double v_projection_23_diff_abs;
		
		double landa1;
		double landa2;
		double landa3;
		
		double crossV1_V2_abs;
		double crossV1_V3_abs;
		double crossV2_V3_abs;
		
		double slip;		 // number of slips
		
		eeros::math::Vector<3> v_wheel_for_IP_calc;	
		eeros::math::Vector<3> phi_GL_d;	
		
		eeros::math::Vector<3> crossV1_V2;		
		eeros::math::Vector<3> crossV1_V3;		
		eeros::math::Vector<3> crossV2_V3;
		
		eeros::math::Vector<3> a1;		// direction vector
		eeros::math::Vector<3> a2;		
		eeros::math::Vector<3> a3;	
		eeros::math::Vector<3> b1;		// direction vector
		eeros::math::Vector<3> b2;		
		eeros::math::Vector<3> b3;	
		
		eeros::math::Vector<3> e3;
		
		eeros::math::Vector<3>  l_12;
		eeros::math::Vector<3>  l_13;
		eeros::math::Vector<3>  l_23;
		eeros::math::Vector<3>  v_projection_12_diff;
		eeros::math::Vector<3>  v_projection_13_diff;
		eeros::math::Vector<3>  v_projection_23_diff;

		eeros::math::Vector<3> v_l_12;
		eeros::math::Vector<3> v_l_13;
		eeros::math::Vector<3> v_l_23;
		
		eeros::math::Matrix<3,3> v_GR_L_projection_l_12;
		eeros::math::Matrix<3,3> v_GR_L_projection_l_13;
		eeros::math::Matrix<3,3> v_GR_L_projection_l_23;
		eeros::math::Vector<3> last_omega_steer_j;
		
		eeros::math::Matrix<3,3> v_wheel_calc;
		
		eeros::math::Vector<3> last_omega_steer_ij;

		
	};
}

#endif /* __CH_NTB_OMNIMOBOT_JACOBI_HPP */