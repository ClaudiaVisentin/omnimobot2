// import standard c library (atoi, atof, ...)
#include <cstdlib>
#include <cmath>

// import IO library (cout, endl, files,  ....)
#include <iostream>
using namespace std;

#include <fstream>

#include <omnimobot/Jacobi.hpp>

using namespace omnimobot;
using namespace eeros::math;



// Funktion um Matrix zu printen
template < int N, int M, typename T >
void print(Matrix<N,M,T> A, int indent = 1)
{
	for (int n = 1; n <= N; n++)
	{
		for (int i = 0; i < indent; i++) cout << '\t';
		for (int m = 1; m <= M; m++)
		{
			if (m > 1) cout << '\t';
			cout << A(n,m);
		}
		cout << endl;
	}
}


int main(int argc, char *argv[])
{
	cout << "test-Jacobi started" << endl;

	double dt = 1.0/1000;
	double laenge = 0.392598183;
	
	double dExcentri = 0.0965049;
	double rRad = 0.05;
	
	Vector3 phi_j_0;
	phi_j_0(0) = 0;
	phi_j_0(1) = 0;
	phi_j_0(2) = 0; 
	
	Vector3 phi_ij_0;
	phi_ij_0(0) = 0;
	phi_ij_0(1) = 0;
	phi_ij_0(2) = 0; 
	
	
	Jacobi jacobi();
	
	
	Vector3 v_l1;
	v_l1(0) = 3;
	v_l1(1) = 34234;
	v_l1(2) = 54688;
	
	Vector6 omega_R2;
	jacobi.calculateInvJacobiLtoR(omega_R2, v_l1, dt);
	
//	cout << omega_R(0) << endl;
	cout << "omega_weel:" << endl;
	cout << '\t' << omega_R2(1) << endl;
//	cout << omega_R(2) << endl;
//	cout << omega_R(3) << endl;
	cout << "omega_steer:" << endl;
	cout << '\t' << omega_R2(4) << endl;
//	cout << omega_R(5) << endl;
	cout << "beta:" << endl;
	cout << '\t'  << jacobi.phi_LR_ij(1) << endl;
	


	
	Vector6 omega_R3;
	omega_R3(0) = 0.2;
	omega_R3(1) = 0.3;
	omega_R3(2) = 2.5;
	omega_R3(3) = 0.25;
	omega_R3(4) = 1.6;
	omega_R3(5) = 0.8;
	
	Vector3 v_l2;
	jacobi.calculateJacobiRtoL(omega_R3, v_l2, dt);
	
//	cout << omega_R(0) << endl;
	cout << "v_lokalx:" << endl;
	cout << '\t' << v_l2(0) << endl;
//	cout << omega_R(2) << endl;
//	cout << omega_R(3) << endl;
	cout << "v_lokaly:" << endl;
	cout << '\t' << v_l2(1) << endl;
//	cout << omega_R(5) << endl;
	cout << "omegaTheta:" << endl;
	cout << '\t' << v_l2(2) << endl;
	cout << "beta_2:" << endl;
	cout << '\t'  << jacobi.phi_LR_j(1) << endl;
	
//	print(omega_R);

	std::ofstream file;
	file.open("/home/ntb/work/transit/FuerMatlab/lokal_ijacobi.txt");

	std::ofstream fileo;
	fileo.open("/home/ntb/work/transit/FuerMatlab/.txt");
	
	
	const double vmin = -0.1;
	const double vmax = 0.1;
	const double thetamin = -0.1;
	const double thetamax = M_PI;
	const double phimin = 0;
	const double phimax = 2*M_PI;
	const double delta_phi = 4;
	const double delta_theta = 2;
	const double delta_v = 0.05;
	
	
	Vector3 v_l;
	Vector3 phi;
	for (v_l(0) = vmin; v_l(0) <= vmax; v_l(0) += delta_v)
	{
		for (v_l(1) = vmin; v_l(1) <= vmax; v_l(1) += delta_v)
		{
			for (v_l(2) = thetamin; v_l(2) <= thetamax; v_l(2) += delta_theta)
			{
				for (phi(0) = phimin; phi(0) <= phimax; phi(0) += delta_phi)
				{
					for (phi(1) = phimin; phi(1) <= phimax; phi(1) += delta_phi)
					{
						for (phi(2) = phimin; phi(2) <= phimax; phi(2) += delta_phi)
						{
		
							file << v_l(0);
							file << '\t' << v_l(1);
							file << '\t' << v_l(2);
							
							file << '\t' << phi(0);
							file << '\t' << phi(1);
							file << '\t' << phi(2);
							file << endl;
							
							Vector6 omega_R;
							
							jacobi.calculateInvJacobiLtoR(omega_R, v_l, phi);
				
							fileo << omega_R(0);
							fileo << '\t' << omega_R(1);
							fileo << '\t' << omega_R(2);
							fileo << '\t' << omega_R(3);
							fileo << '\t' << omega_R(4);
							fileo << '\t' << omega_R(5);
							
							fileo << '\t' << jacobi.phi_LR_ij(1);
							fileo << endl;
						}
					}
				}
			}
		}
	}

	
	file.close();
	fileo.close();
	
	std::ofstream file_o_j;
	file_o_j.open("/home/ntb/work/transit/omega_jacobi.txt");

	std::ofstream file_vl_j;
	file_vl_j.open("/home/ntb/work/transit/lokal_jacobi.txt");
	
	
	const double omega_w1_min = -1;
	const double omega_w1_max = 1;
	const double omega_w2_min = -1;
	const double omega_w2_max = 1;
	const double omega_w3_min = -1;
	const double omega_w3_max = 1;
	const double omega_s1_min = -M_PI;
	const double omega_s1_max = M_PI;
	const double omega_s2_min = -M_PI;
	const double omega_s2_max = M_PI;
	const double omega_s3_min = -M_PI;
	const double omega_s3_max = M_PI;
	const double omega_w_delay = 0.8;
	const double omega_s_delay = M_PI/3;

	
	Vector6 omega_j;
	for (omega_j(0) = omega_w1_min; omega_j(0) <= omega_w1_max; omega_j(0) += omega_w_delay)
	{
		for (omega_j(1) = omega_w2_min; omega_j(1) <= omega_w2_max; omega_j(1) += omega_w_delay)
		{
			for (omega_j(2) = omega_w3_min; omega_j(2) <= omega_w3_max; omega_j(2) += omega_w_delay)
			{
				for (omega_j(3) = omega_s1_min; omega_j(3) <= omega_s1_max; omega_j(3) += omega_s_delay)
				{
					for (omega_j(4) = omega_s2_min; omega_j(4) <= omega_s2_max; omega_j(4) += omega_s_delay)
					{
						for (omega_j(5) = omega_s3_min; omega_j(5) <= omega_s3_max; omega_j(5) += omega_s_delay)
						{
							file_o_j << omega_j(0);
							file_o_j << '\t' << omega_j(1);
							file_o_j << '\t' << omega_j(2);
							file_o_j << '\t' << omega_j(3);
							file_o_j << '\t' << omega_j(4);
							file_o_j << '\t' << omega_j(5);
							file_o_j << endl;
							
							Vector3 v_lokal_j;
							
							jacobi.calculateJacobiRtoL(omega_j,v_lokal_j,dt);
							
							file_vl_j << v_lokal_j(0);
							file_vl_j << '\t' << v_lokal_j(1);
							file_vl_j << '\t' << v_lokal_j(2);
							
							file_vl_j << '\t' << jacobi.phi_LR_j(1);
							file_vl_j << endl;
						}
					}
				}
			}
		}
	}

	
	file_o_j.close();
	file_vl_j.close();

	cout << "test-Jacobi finished" << endl;

	return EXIT_SUCCESS;
}






