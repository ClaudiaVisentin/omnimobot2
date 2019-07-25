#include <omnimobot/Jacobi.hpp>
#include <omnimobot/constants.hpp>
#include <cmath>
#include <iostream>

using namespace omnimobot;
using namespace eeros::math;

Jacobi::Jacobi():
slip(0.0),
crossV1_V2_abs(0.0),
crossV1_V3_abs(0.0),
crossV2_V3_abs(0.0),
landa1(0.0),
landa2(0.0),
landa3(0.0),
l_1_abs(0.0),
l_2_abs(0.0),
l_3_abs(0.0),
l_12_abs(0.0),
l_13_abs(0.0),
l_23_abs(0.0),
v_l_12_abs(0.0),
v_l_13_abs(0.0),
v_l_23_abs(0.0)
{ 
	phi_LR_j.zero();
	phi_LR_ij.zero();
	
	last_omega_steer_j.zero();
	last_omega_steer_ij.zero();
	
	l_i_0(0) = deltax1;
	l_i_0(1) = deltay1;
	l_i_0(2) = 0.0;
	
	l_i_1(0) = deltax2;
	l_i_1(1) = deltay2;
	l_i_1(2) = 0.0;

	l_i_2(0) = deltax3;
	l_i_2(1) = deltay3;
	l_i_2(2) = 0.0;
	
	l_1_abs = sqrt(l_i_0(0)*l_i_0(0) + l_i_0(1)*l_i_0(1));
	l_2_abs = sqrt(l_i_1(0)*l_i_1(0) + l_i_1(1)*l_i_1(1));
	l_3_abs = sqrt(l_i_2(0)*l_i_2(0) + l_i_2(1)*l_i_2(1));
	
	// l_12 is the distance between Wheel 1 and 2
	l_12 = l_i_1 - l_i_0;
	l_12_abs = sqrt(l_12(0)*l_12(0) + l_12(1)*l_12(1));
	
	// l_13 is the distance between Wheel 1 and 3
    l_13 = l_i_2 - l_i_0; 
    l_13_abs = sqrt(l_13(0)*l_13(0) + l_13(1)*l_13(1));
	
	// l_23 is the distance between Wheel 2 and 3
    l_23 = l_i_2 - l_i_1; 
    l_23_abs = sqrt(l_23(0)*l_23(0) + l_23(1)*l_23(1));
	
	v_GR_L_projection_l_12.zero();
	v_GR_L_projection_l_13.zero();
	v_GR_L_projection_l_23.zero();
	
	v_projection_12_diff.zero();
	v_projection_13_diff.zero();
	v_projection_23_diff.zero();
	
	v_projection_12_diff_abs = 0.0;
	v_projection_13_diff_abs = 0.0;
	v_projection_23_diff_abs = 0.0;
	
	v_l_12.zero();
	v_l_13.zero();
	v_l_23.zero();
	
	a1.zero();
	a2.zero();
	a3.zero();
	b1.zero();
	b2.zero();
	b3.zero();
	
	e3 << 0.0,
		  0.0,
		  1.0;
		  
	crossV1_V2.zero();
	crossV1_V3.zero();
	crossV2_V3.zero();
	
	v_wheel_for_IP_calc.zero();
	phi_GL_d.zero();
	
	d_theta.zero();
	v_GR_L_i.zero();
	v_GRx_R.zero();
	v_GRy_R.zero();
	v_GR_R.zero();
	Rot_RtoL.zero();
	v_wheel_calc.zero();
// 	jacobi_aug_6x3.zero();
// 	jacobi_aug_4x3.zero();
// 	pseudo_jacobi_LPT_3x6.zero();
// 	pseudo_jacobi_LPT_3x4.zero();
	tmpMatrix.zero();
	omega_4.zero();
}


Jacobi::~Jacobi() { }



Vector3 Jacobi::getVeloLocal(const Vector6& omega, Vector3& v_local, const eeros::math::Vector3 phi_LR_j)
{
	calculateJacobiRtoL(omega, v_local, phi_LR_j);
	
	return v_local;
}


Vector6 Jacobi::getOmega( Vector6& omega, const Vector3& v_global, const Vector3 phi_LR, const double phi_GL)
{
	calculateInvJacobiGtoR(omega, v_global, phi_LR, phi_GL);
	
	return omega;
}


void Jacobi::calculateJacobiRtoL(const Vector6& omega, Vector3& v_local, const Vector3 phi_LR_j)
{
	
	Vector<3> cosPhi_LR;
	cosPhi_LR(0) = cos(phi_LR_j(0));
	cosPhi_LR(1) = cos(phi_LR_j(1));
	cosPhi_LR(2) = cos(phi_LR_j(2));
	
	Vector<3> sinPhi_LR;
	sinPhi_LR(0) = sin(phi_LR_j(0));
	sinPhi_LR(1) = sin(phi_LR_j(1));
	sinPhi_LR(2) = sin(phi_LR_j(2));
	
	 // Vector describt all velocity of the three weels
	v_GRx_R(0) = omega(0)*radiusWheel;
	v_GRx_R(1) = omega(1)*radiusWheel;
	v_GRx_R(2) = omega(2)*radiusWheel;
	
	 // Vector describt all velocity of the three steers, (minus wie bei inverse)!!!
	v_GRy_R(0) = -omega(3)*steerOffset;
	v_GRy_R(1) = -omega(4)*steerOffset;
	v_GRy_R(2) = -omega(5)*steerOffset;
	
	// i wheels
	for (int i = 0 ; i<3; i++)
	{
		v_GR_R(0) = v_GRx_R(i);
		v_GR_R(1) = v_GRy_R(i);
	
		Rot_RtoL(0,0) = cosPhi_LR(i);
		Rot_RtoL(1,0) = sinPhi_LR(i);
		
		Rot_RtoL(0,1) = -sinPhi_LR(i);
		Rot_RtoL(1,1) = cosPhi_LR(i);

		Rot_RtoL(2,2) = 1.0;
		
		// Transformation R to L 
		v_GR_L_i(i,0) = Rot_RtoL(0,0)*v_GR_R(0)+Rot_RtoL(0,1)*v_GR_R(1)+Rot_RtoL(0,2)*v_GR_R(2);
		v_GR_L_i(i,1) = Rot_RtoL(1,0)*v_GR_R(0)+Rot_RtoL(1,1)*v_GR_R(1)+Rot_RtoL(1,2)*v_GR_R(2);
		v_GR_L_i(i,2) = Rot_RtoL(2,0)*v_GR_R(0)+Rot_RtoL(2,1)*v_GR_R(1)+Rot_RtoL(2,2)*v_GR_R(2);
	}
	
	// Cleaner
	
	
	// All wheels velocities are projected to the distances between the wheels
	for (int m = 0 ; m<3; m++) // m: which wheel
	{
		for (int n = 0 ; n<2; n++) // n: x-Value, y-Value, z-Value
		{
			v_GR_L_projection_l_12(m,n) = ( (l_12(0) * v_GR_L_i(m,0) + l_12(1) * v_GR_L_i(m,1)) / (l_12_abs*l_12_abs) ) * l_12(n);
			v_GR_L_projection_l_13(m,n) = ( (l_13(0) * v_GR_L_i(m,0) + l_13(1) * v_GR_L_i(m,1)) / (l_13_abs*l_13_abs) ) * l_13(n);
			v_GR_L_projection_l_23(m,n) = ( (l_23(0) * v_GR_L_i(m,0) + l_23(1) * v_GR_L_i(m,1)) / (l_23_abs*l_23_abs) ) * l_23(n);
		}
	}
	
	
	// Control the projection velocities
	
	
	int badVelocity = 0;
	
	for (int n =0; n<2; n++)
	{
		v_projection_12_diff(n) = v_GR_L_projection_l_12(0,n) - v_GR_L_projection_l_12(1,n); 
		v_projection_13_diff(n) = v_GR_L_projection_l_13(0,n) - v_GR_L_projection_l_13(2,n); 
		v_projection_23_diff(n) = v_GR_L_projection_l_23(1,n) - v_GR_L_projection_l_23(2,n); 
	}
	
	v_projection_12_diff_abs = sqrt( v_projection_12_diff(0)*v_projection_12_diff(0) + v_projection_12_diff(1)*v_projection_12_diff(1) );
	v_projection_13_diff_abs = sqrt( v_projection_13_diff(0)*v_projection_13_diff(0) + v_projection_13_diff(1)*v_projection_13_diff(1) );
	v_projection_23_diff_abs = sqrt( v_projection_23_diff(0)*v_projection_23_diff(0) + v_projection_23_diff(1)*v_projection_23_diff(1) );
	
	// Control the projection velocities to l_12
	if (v_projection_12_diff_abs > permissibleDifference)
	{
		badVelocity = 1;
// 		throw -10; // TODO
	}	
	
	// Control the projection velocities to l_13
	if (v_projection_13_diff_abs > permissibleDifference)
	{
		badVelocity = badVelocity + 2;
// 		throw -11; // TODO
	}	
	
	// Control the projection velocities to l_23
	if (v_projection_23_diff_abs > permissibleDifference)
	{
		badVelocity = badVelocity + 4;
// 		throw -12; // TODO
	}	
	
	
	// bad velocity is sorted out old kinematik)

	if( badVelocity == 3 )  // v_wheel_1 is bad
	{
		slip = slip + 1.0;
		
// 		jacobi_aug_4x3 << cosPhi_LR(1)/radiusWheel, sinPhi_LR(1)/radiusWheel, (sinPhi_LR(1)*deltax2-cosPhi_LR(1)*deltay2)/radiusWheel,
// 		              cosPhi_LR(2)/radiusWheel, sinPhi_LR(2)/radiusWheel, (sinPhi_LR(2)*deltax3-cosPhi_LR(2)*deltay3)/radiusWheel,
// 		              -1.0*(-sinPhi_LR(1)/steerOffset), -1.0*(cosPhi_LR(1)/steerOffset), -1.0*((-sinPhi_LR(1)*-deltay2 + cosPhi_LR(1)*deltax2)/steerOffset),
// 		              -1.0*(-sinPhi_LR(2)/steerOffset), -1.0*(cosPhi_LR(2)/steerOffset), -1.0*((-sinPhi_LR(2)*-deltay3 + cosPhi_LR(2)*deltax3)/steerOffset);
// 
// 		
// 		tmpMatrix = jacobi_aug_4x3.transpose()*jacobi_aug_4x3;
// 		
// 		pseudo_jacobi_LPT_3x4 = !tmpMatrix * jacobi_aug_4x3.transpose();
// 		
// 		omega_4 << omega(1), omega(2), omega(4), omega(5);
// 		
// 		v_local = pseudo_jacobi_LPT_3x4 * omega_4;
					  
		for (int n=0; n<2; n++) 
		{
			v_l_12(n) = v_GR_L_projection_l_12(1,n);
			v_l_13(n) = v_GR_L_projection_l_13(2,n);
			v_l_23(n) = (v_GR_L_projection_l_23(1,n) + v_GR_L_projection_l_23(2,n)) / 2.0; 
		}
	}
	else if ( badVelocity == 5 )  // v_wheel_2 is bad
	{
/*		slip = slip + 1.0;
		
		jacobi_aug_4x3 << cosPhi_LR(0)/radiusWheel, sinPhi_LR(0)/radiusWheel, (sinPhi_LR(0)*deltax1-cosPhi_LR(0)*deltay1)/radiusWheel,
					  cosPhi_LR(2)/radiusWheel, sinPhi_LR(2)/radiusWheel, (sinPhi_LR(2)*deltax3-cosPhi_LR(2)*deltay3)/radiusWheel,
					  -1.0*(-sinPhi_LR(0)/steerOffset), -1.0*(cosPhi_LR(0)/steerOffset), -1.0*((-sinPhi_LR(0)*-deltay1 + cosPhi_LR(0)*deltax1)/steerOffset),
				      -1.0*(-sinPhi_LR(2)/steerOffset), -1.0*(cosPhi_LR(2)/steerOffset), -1.0*((-sinPhi_LR(2)*-deltay3 + cosPhi_LR(2)*deltax3)/steerOffset);

		tmpMatrix = jacobi_aug_4x3.transpose()*jacobi_aug_4x3;
		
		pseudo_jacobi_LPT_3x4 = !tmpMatrix * jacobi_aug_4x3.transpose();
		
		omega_4 << omega(0), omega(2), omega(3), omega(5);
		
		v_local = pseudo_jacobi_LPT_3x4 * omega_4;		*/	  
					  
					  
		for (int n=0; n<2; n++) 
		{
			v_l_12(n) = v_GR_L_projection_l_12(0,n);
			v_l_13(n) = (v_GR_L_projection_l_13(0,n) + v_GR_L_projection_l_13(2,n)) / 2.0; 
			v_l_23(n) = v_GR_L_projection_l_23(2,n);
		}
	}
	else if ( badVelocity == 6 )  // v_wheel_3 is bad
	{
// 		slip = slip + 1.0;
		
// 		jacobi_aug_4x3 << cosPhi_LR(0)/radiusWheel, sinPhi_LR(0)/radiusWheel, (sinPhi_LR(0)*deltax1-cosPhi_LR(0)*deltay1)/radiusWheel,
// 				      cosPhi_LR(1)/radiusWheel, sinPhi_LR(1)/radiusWheel, (sinPhi_LR(1)*deltax2-cosPhi_LR(1)*deltay2)/radiusWheel,
// 				      -1.0*(-sinPhi_LR(0)/steerOffset), -1.0*(cosPhi_LR(0)/steerOffset), -1.0*((-sinPhi_LR(0)*-deltay1 + cosPhi_LR(0)*deltax1)/steerOffset),
// 				      -1.0*(-sinPhi_LR(1)/steerOffset), -1.0*(cosPhi_LR(1)/steerOffset), -1.0*((-sinPhi_LR(1)*-deltay2 + cosPhi_LR(1)*deltax2)/steerOffset);
// 				      
// 		tmpMatrix = jacobi_aug_4x3.transpose()*jacobi_aug_4x3;
// 		
// 		pseudo_jacobi_LPT_3x4 = !tmpMatrix * jacobi_aug_4x3.transpose();
// 		
// 		omega_4 << omega(0), omega(1), omega(3), omega(4);
// 		
// 		v_local = pseudo_jacobi_LPT_3x4 * omega_4;
					  
					  
		for (int n=0; n<2; n++) 
		{
			v_l_12(n) = (v_GR_L_projection_l_12(0,n) + v_GR_L_projection_l_12(1,n)) / 2.0;
			v_l_13(n) = v_GR_L_projection_l_13(0,n);
			v_l_23(n) = v_GR_L_projection_l_23(1,n);
		}
	}
	else 
	{
// 		jacobi_aug_6x3 << cosPhi_LR(0)/radiusWheel, sinPhi_LR(0)/radiusWheel, (sinPhi_LR(0)*deltax1-cosPhi_LR(0)*deltay1)/radiusWheel,
// 				      cosPhi_LR(1)/radiusWheel, sinPhi_LR(1)/radiusWheel, (sinPhi_LR(1)*deltax2-cosPhi_LR(1)*deltay2)/radiusWheel,
// 				      cosPhi_LR(2)/radiusWheel, sinPhi_LR(2)/radiusWheel, (sinPhi_LR(2)*deltax3-cosPhi_LR(2)*deltay3)/radiusWheel,
// 				      -1.0*(-sinPhi_LR(0)/steerOffset), -1.0*(cosPhi_LR(0)/steerOffset), -1.0*((-sinPhi_LR(0)*-deltay1 + cosPhi_LR(0)*deltax1)/steerOffset),
// 				      -1.0*(-sinPhi_LR(1)/steerOffset), -1.0*(cosPhi_LR(1)/steerOffset), -1.0*((-sinPhi_LR(1)*-deltay2 + cosPhi_LR(1)*deltax2)/steerOffset),
// 				      -1.0*(-sinPhi_LR(2)/steerOffset), -1.0*(cosPhi_LR(2)/steerOffset), -1.0*((-sinPhi_LR(2)*-deltay3 + cosPhi_LR(2)*deltax3)/steerOffset);
// 
// 		tmpMatrix = jacobi_aug_6x3.transpose()*jacobi_aug_6x3;
// 		
// 		pseudo_jacobi_LPT_3x6 = !tmpMatrix * jacobi_aug_6x3.transpose();
// 		
// 		v_local = pseudo_jacobi_LPT_3x6 * omega;
		
		
		// Two velocities create a mathematically correct velocity
		for (int n=0; n<2; n++)
		{
			v_l_12(n) = (v_GR_L_projection_l_12(0,n) + v_GR_L_projection_l_12(1,n)) / 2.0; 
			v_l_13(n) = (v_GR_L_projection_l_13(0,n) + v_GR_L_projection_l_13(2,n)) / 2.0; 
			v_l_23(n) = (v_GR_L_projection_l_23(1,n) + v_GR_L_projection_l_23(2,n)) / 2.0; 
		}
	}
	
	// new wheel velocities 
	
	v_l_12_abs = sqrt(v_l_12(0)*v_l_12(0) + v_l_12(1)*v_l_12(1));
	v_l_13_abs = sqrt(v_l_13(0)*v_l_13(0) + v_l_13(1)*v_l_13(1));
	v_l_23_abs = sqrt(v_l_23(0)*v_l_23(0) + v_l_23(1)*v_l_23(1));
	
	// v wheel 1
	a1 = Vector3::crossProduct( v_l_12, e3); // v_l_12 = v1 (in doku)
	b1 = Vector3::crossProduct(-v_l_13, e3); // v_l_13 = v2 (in doku) 
	
	// v wheel 2
	a2 = Vector3::crossProduct( v_l_23, e3); // v_l_23 = v1
	b2 = Vector3::crossProduct(-v_l_12, e3); // v_l_12 = v2
	
	// v wheel 3
	a3 = Vector3::crossProduct( v_l_13, e3); // v_l_13 = v1 
	b3 = Vector3::crossProduct(-v_l_23, e3); // v_l_23 = v2 
	
	
	// standstill
	if (v_l_12_abs <= minVelocity && v_l_13_abs <= minVelocity && v_l_23_abs <= minVelocity) 
	{
		v_local(0) = 0.0;
		v_local(1) = 0.0;
		v_local(2) = 0.0;
	}
	else 
	{
		//  the robot moves orthogonal to l_12
		if ( v_l_12_abs <= minVelocity && v_l_13_abs >= minVelocity && v_l_23_abs >= minVelocity) 
		{

// 			// v wheel 3
			landa3 = (b3(0)*(v_l_13(1) - v_l_23(1)) - b3(1)*(v_l_13(0) - v_l_23(0))) / (a3(0)*b3(1)-a3(1)*b3(0)); // landa from v_l_13
			
			if (std::isnan(landa3))
			{
				std::cout << " !!!!!!!!!! v_l_12_abs <= minVelocity     a3(0): " <<a3(0)<<"  a3(1): "<<a3(1)<< "  b3(0): " <<b3(0)<<"  b3(1): "<<b3(1)<<"  v_l_12_abs: "<< v_l_12_abs <<"  v_l_23_abs: "<< v_l_23_abs<<"  v_l_13_abs: "<< v_l_13_abs<< std::endl;
			}
	
// 			// The actual velocity of the robot calc
			v_local(0) = v_l_13(0) + landa3 * a3(0);
			v_local(1) = v_l_13(1) + landa3 * a3(1);
			v_local(2) = 0.0;
		}
		else if ( v_l_23_abs <= minVelocity && v_l_12_abs >= minVelocity && v_l_13_abs >= minVelocity ) // the robot moves orthogonal to l_23
		{

// 			// v wheel 1			
			landa1 = (b1(0)*(v_l_12(1) - v_l_13(1)) - b1(1)*(v_l_12(0) - v_l_13(0))) / (a1(0)*b1(1)-a1(1)*b1(0)); // in doku landa11 from v_l_12
			
			if (std::isnan(landa1))
			{
				std::cout << " !!!!!!!!!!  v_l_23_abs <= minVelocity     a1(0): " <<a1(0)<<"  a1(1): "<<a1(1)<< "  b1(0): " <<b1(0)<<"  b1(1): "<<b1(1)<< "  a3(0): " <<a3(0)<<"  a3(1): "<<a3(1)<<"  v_l_12_abs: "<< v_l_12_abs <<"  v_l_23_abs: "<< v_l_23_abs<<"  v_l_13_abs: "<< v_l_13_abs<< std::endl;
			}
			

// 			// The actual velocity of the robot calc
			v_local(0) = v_l_12(0) + landa1 * a1(0);
			v_local(1) = v_l_12(1) + landa1 * a1(1);
			v_local(2) = 0.0;
		}
		else if (v_l_13_abs <= minVelocity && v_l_12_abs >= minVelocity  && v_l_23_abs >= minVelocity) // the robot moves orthogonal to l_13
		{

			// v wheel 2
			a2 = Vector3::crossProduct( v_l_23, e3); // v_l_23 = v1
			b2 = Vector3::crossProduct(-v_l_12, e3); // v_l_12 = v2
			
			landa2 = (b2(0)*(v_l_23(1) - v_l_12(1)) - b2(1)*(v_l_23(0) - v_l_12(0))) / (a2(0)*b2(1)-a2(1)*b2(0)); // landa from v_l_23
			
			if (std::isnan(landa2))
			{
				std::cout << " !!!!!!!!!! if v_l_13_abs <= minVelocity     a2(0): " <<a2(0)<<"  a2(1): "<<a2(1)<< "  b2(0): " <<b2(0)<<"  b2(1): "<<b2(1)<<"  v_l_12_abs: "<< v_l_12_abs <<"  v_l_23_abs: "<< v_l_23_abs<<"  v_l_13_abs: "<< v_l_13_abs<< std::endl;
			}
 			
// 			// The actual velocity of the robot calc
			v_local(0) = v_l_23(0) + landa2 * a2(0);
			v_local(1) = v_l_23(1) + landa2 * a2(1);
			v_local(2) = 0.0;
		}
		else if (v_l_13_abs <= minVelocity && v_l_12_abs <= minVelocity  && v_l_23_abs >= minVelocity) // rotates about the axis of the wheel 1
		{
			Vector<3> v_local_tmp;
			v_local_tmp = 2.0/3.0 * v_l_23;
			
			phi_GL_d = Vector3::crossProduct(-l_i_0,v_local_tmp) / (l_1_abs*l_1_abs);
			
			v_local(0) = v_local_tmp(0);
			v_local(1) = v_local_tmp(1);
			v_local(2) = phi_GL_d(2);
			
		}
		else if (v_l_13_abs >= minVelocity && v_l_12_abs <= minVelocity  && v_l_23_abs <= minVelocity) // rotates about the axis of the wheel 2
		{
			Vector<3> v_local_tmp;
			v_local_tmp = 2.0/3.0 * v_l_13;
			
			phi_GL_d = Vector3::crossProduct(-l_i_1,v_local_tmp) / (l_2_abs*l_2_abs);
			
			v_local(0) = v_local_tmp(0);
			v_local(1) = v_local_tmp(1);
			v_local(2) = phi_GL_d(2);
			
		}
		else if (v_l_13_abs <= minVelocity && v_l_12_abs >= minVelocity  && v_l_23_abs <= minVelocity) // rotates about the axis of the wheel 3
		{
			Vector<3> v_local_tmp;
			v_local_tmp = 2.0/3.0 * v_l_12;
			
			phi_GL_d = Vector3::crossProduct(-l_i_2,v_local_tmp) / (l_3_abs*l_3_abs);
			
			v_local(0) = v_local_tmp(0);
			v_local(1) = v_local_tmp(1);
			v_local(2) = phi_GL_d(2);
			
		}
		else if (v_l_12_abs >= minVelocity && v_l_13_abs >= minVelocity && v_l_23_abs >= minVelocity)
		{
		
// 			// v wheel 1
			
			landa1 = (b1(0)*(v_l_12(1) - v_l_13(1)) - b1(1)*(v_l_12(0) - v_l_13(0))) / (a1(0)*b1(1)-a1(1)*b1(0)); // in doku landa11
			
// 			// v wheel 2

			landa2 = (b2(0)*(v_l_23(1) - v_l_12(1)) - b2(1)*(v_l_23(0) - v_l_12(0))) / (a2(0)*b2(1)-a2(1)*b2(0));
			
// 			// v wheel 3
			
			landa3 = (b3(0)*(v_l_13(1) - v_l_23(1)) - b3(1)*(v_l_13(0) - v_l_23(0))) / (a3(0)*b3(1)-a3(1)*b3(0));
			
			if (std::isnan(landa1))
			{
				std::cout << " !!!!!!!!!! else  a1(0): " <<a1(0)<<"  a1(1): "<<a1(1)<< "  b1(0): " <<b1(0)<<"  b1(1): "<<b1(1)<< "  a3(0): " <<a3(0)<<"  a3(1): "<<a3(1)<<"  v_l_12_abs: "<< v_l_12_abs <<"  v_l_23_abs: "<< v_l_23_abs<<"  v_l_13_abs: "<< v_l_13_abs<< std::endl;
			}
			
			if (std::isnan(landa2))
			{
				std::cout << " !!!!!!!!!! else  a2(0): " <<a2(0)<<"  a2(1): "<<a2(1)<< "  b2(0): " <<b2(0)<<"  b2(1): "<<b2(1)<<"  v_l_12_abs: "<< v_l_12_abs <<"  v_l_23_abs: "<< v_l_23_abs<<"  v_l_13_abs: "<< v_l_13_abs<< std::endl;
			}
			
			if (std::isnan(landa3))
			{
				std::cout << " !!!!!!!!!! else a3(0): " <<a3(0)<<"  a3(1): "<<a3(1)<< "  b3(0): " <<b3(0)<<"  b3(1): "<<b3(1)<<"  v_l_12_abs: "<< v_l_12_abs <<"  v_l_23_abs: "<< v_l_23_abs<<"  v_l_13_abs: "<< v_l_13_abs<< std::endl;
			}
			
			// new wheel velocities calc

			for (int n = 0 ; n<2; n++)
			{ // n: x-Value, y-Value, z-Value
				v_wheel_calc(0,n) = v_l_12(n) + landa1 * a1(n);
				v_wheel_calc(1,n) = v_l_23(n) + landa2 * a2(n);
				v_wheel_calc(2,n) = v_l_13(n) + landa3 * a3(n);
			}
				
			// The actual velocity of the robot calc
			v_local(0) = (v_wheel_calc(0,0) + v_wheel_calc(1,0) + v_wheel_calc(2,0)) / 3.0;
			v_local(1) = (v_wheel_calc(0,1) + v_wheel_calc(1,1) + v_wheel_calc(2,1)) / 3.0;
				
			
			// Instantaneous pole (IP)
			
			// exist a IP
			
			Vector<3> tmp1;
			Vector<3> tmp2;
			Vector<3> tmp3;
			
			tmp1.zero();
			tmp2.zero();
			tmp3.zero();
			
			for (int n = 0 ; n<2; n++)
			{ // n: x-Value, y-Value, z-Value
				tmp1(n) = v_wheel_calc(0,n);
				tmp2(n) = v_wheel_calc(1,n);
				tmp3(n) = v_wheel_calc(2,n);
			}

			crossV1_V2 = Vector3::crossProduct(tmp1,tmp2);
			crossV1_V3 = Vector3::crossProduct(tmp1,tmp3);
			crossV2_V3 = Vector3::crossProduct(tmp2,tmp3);
			
			crossV1_V2_abs = sqrt(crossV1_V2(2)*crossV1_V2(2));
			crossV1_V3_abs = sqrt(crossV1_V3(2)*crossV1_V3(2));
			crossV2_V3_abs = sqrt(crossV2_V3(2)*crossV2_V3(2));
			
			// not any Ip
			if (crossV1_V2_abs < minVelocity && crossV1_V3_abs < minVelocity && crossV2_V3_abs < minVelocity)
			{ 
				v_local(2) = 0.0;			
			}
			else 
			{
				if(badVelocity == 3)
				{ // v1 is bad
					v_wheel_for_IP_calc = tmp3 - tmp2;
					phi_GL_d = (Vector3::crossProduct(l_23,v_wheel_for_IP_calc)) / (l_23_abs*l_23_abs);
				}
				else if(badVelocity == 5)
				{ // v2 is bad
					v_wheel_for_IP_calc = tmp3 - tmp1;
					phi_GL_d = (Vector3::crossProduct(l_13,v_wheel_for_IP_calc)) / (l_13_abs*l_13_abs);
				}
				else 
				{ // v3 is bad
					v_wheel_for_IP_calc = tmp2 - tmp1;
					phi_GL_d = (Vector3::crossProduct(l_12,v_wheel_for_IP_calc)) / (l_12_abs*l_12_abs);
				}

				v_local(2) = phi_GL_d(2);
			}
		}
		else // gear backlash
		{
			std::cout << " !!!!!!!!!! gear backlash !!!!!!!  " << std::endl;
			
			v_local(0) = 0.0;
			v_local(1) = 0.0;
			v_local(2) = 0.0;
		}
	}
	
}



void Jacobi::calculateInvJacobiGtoR( Vector6& omega, const Vector3& v_global, const Vector3 phi_LR, const double phi_GL)
{
	
	double cosPhi_LR_GL_0 = cos(phi_LR(0)+phi_GL);
	double cosPhi_LR_GL_1 = cos(phi_LR(1)+phi_GL);
	double cosPhi_LR_GL_2 = cos(phi_LR(2)+phi_GL);
	
	double sinPhi_LR_GL_0 = sin(phi_LR(0)+phi_GL);
	double sinPhi_LR_GL_1 = sin(phi_LR(1)+phi_GL);
	double sinPhi_LR_GL_2 = sin(phi_LR(2)+phi_GL);
	
	double cosPhi_LR_0 = cos(phi_LR(0));
	double cosPhi_LR_1 = cos(phi_LR(1));
	double cosPhi_LR_2 = cos(phi_LR(2));
	
	double sinPhi_LR_0 = sin(phi_LR(0));
	double sinPhi_LR_1 = sin(phi_LR(1));
	double sinPhi_LR_2 = sin(phi_LR(2));

	omega(0) = v_global(0)*cosPhi_LR_GL_0/radiusWheel + v_global(1)*sinPhi_LR_GL_0/radiusWheel + v_global(2)*(sinPhi_LR_0*deltax1-cosPhi_LR_0*deltay1)/radiusWheel;
	omega(1) = v_global(0)*cosPhi_LR_GL_1/radiusWheel + v_global(1)*sinPhi_LR_GL_1/radiusWheel + v_global(2)*(sinPhi_LR_1*deltax2-cosPhi_LR_1*deltay2)/radiusWheel;
	omega(2) = v_global(0)*cosPhi_LR_GL_2/radiusWheel + v_global(1)*sinPhi_LR_GL_2/radiusWheel + v_global(2)*(sinPhi_LR_2*deltax3-cosPhi_LR_2*deltay3)/radiusWheel;
	
	// minus beachten!!
	omega(3) = -1.0*(-v_global(0)*sinPhi_LR_GL_0/steerOffset + v_global(1)*cosPhi_LR_GL_0/steerOffset + v_global(2)*(cosPhi_LR_0*deltax1-sinPhi_LR_0*-deltay1)/steerOffset);
	omega(4) = -1.0*(-v_global(0)*sinPhi_LR_GL_1/steerOffset + v_global(1)*cosPhi_LR_GL_1/steerOffset + v_global(2)*(cosPhi_LR_1*deltax2-sinPhi_LR_1*-deltay2)/steerOffset);
	omega(5) = -1.0*(-v_global(0)*sinPhi_LR_GL_2/steerOffset + v_global(1)*cosPhi_LR_GL_2/steerOffset + v_global(2)*(cosPhi_LR_2*deltax3-sinPhi_LR_2*-deltay3)/steerOffset);

}

double Jacobi::getNumberOfSlips()
{
       return slip;
}










