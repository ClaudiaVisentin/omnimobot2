#include <eeros/core/Runnable.hpp>
#include <omnimobot/control/block/I.hpp>
#include <omnimobot/control/block/InvJacobian.hpp>

#include <iostream>
#include <fstream>

#include <Utils.hpp>

using namespace eeros::math;

template <typename T = double>
class InvJacobiBlockTest {
	public:
		InvJacobiBlockTest() {
			// Verknüpfung invJacobi
			invJacobiBlock.getInVglobal().connect(data);
			invJacobiBlock.getInPhiSteer().connect(dataPhiSteer);
//			integrator.getIn().connect(invJacobiBlock.getOutOmegaSteer());
		}
		
		int run(const char* filepath) {
			std::ifstream file(filepath);
			std::cout << "run ausfürend" << std::endl;
			if (!file.is_open()) return -2;
			
			int line = 0;
			int error = 0;
			uint64_t timestamp = 0;
			bool first = true;
			double lastPhiSteer;
				
				std::ofstream plot;
				plot.open("/home/ntb/work/transit/FuerMatlab/inv_jacobian_test_daten_plot.txt");
			
			while (!file.eof()) {
				line++;
				
				
				if (first){
					lastPhiSteer = 0;
				}
				
				Vector3 in; // vx,vy,d_theta
				Vector2 out; // Daten von Rad 2
				Vector3 phiSteer; // Ausgang des Integrators von Matlab
				phiSteer(0) = 0;
				phiSteer(1) = lastPhiSteer;
				phiSteer(2) = 0;
				
				file >> in(0); // read input data vx
				file >> in(1); // read input data vy
				file >> in(2); // read input data d_theta
				file >> out(0); // rea md reference result omega_weel_rad2
				file >> out(1); // read reference result omega_steer_rad2
				file >> lastPhiSteer; // phi_LR_rad2
				
				if (file.eof()) break;
				
				dataPhiSteer.getSignal().setValue(phiSteer);
				dataPhiSteer.getSignal().setTimestamp(timestamp);
				data.getSignal().setValue(in);
				data.getSignal().setTimestamp(timestamp);
				timestamp += 1000000;
				
				invJacobiBlock.run();
//				integrator.run();
				
//				Vector3 phi_LR = integrator.getOut().getSignal().getValue();
			
				Vector6 omega = invJacobiBlock.getOutOmega().getSignal().getValue();
				
// 				if(line>=0 and line<=10) {
// 					std::cout << line;
// 					std::cout << "\t" << in(0) << "\t" << in(1) << "\t" <<  in(2) << std::endl;
// 					std::cout << "\t" << out(1) <<" berechnet steerRad2: " << omega(4) << std::endl;
// 					std::cout << "\t" << out(0) <<" berechnet weelRad2: " << omega(1) << std::endl << std::endl;
// 					
// 				}
// 				
// 				
// 				if(line>=0 and line<=10) {
// 					std::cout << line;
// 					std::cout << "\t" << in(0) << "\t" << in(1) << "\t" <<  in(2) << std::endl;
// 					std::cout << "\t" << out(1) <<" berechnet steerRad2: " << omega(4) << std::endl;
// 					std::cout << "\t" << out(0) <<" berechnet weelRad2: " << omega(1) << std::endl << std::endl;
// 					
// 					
// 			
// 					plot << out(1) <<'\t' << omega(4);
// 					plot << "\t" << out(0) <<'\t' << omega(1) << std::endl;
// 					
// 					
// 				}

				if(!Utils::compareApprox(out(0),omega(1) , 0.000001)) {
					std::cout << "line " << line << " expecting " << out(0) << " calculated " << omega(1) << std::endl;
					error = 1;
					
				}
				if(!Utils::compareApprox(out(1),omega(4) , 0.000001)) {
					std::cout << "line " << line << " expecting " << out(1) << " calculated " << omega(4) << std::endl;
					error = 2;
// 					
				}
// 				if(!Utils::compareApprox(out(2),phi_LR(1) , 0.0001)) {
// 					
// 					std::cout << "line " << line << " expecting " << out(2) << " calculated " << phi_LR(1) << std::endl;
// 					error = 3;
// 					
// 				}
				first = false;
			}
			
			file.close();
			plot.close();
			std::cout << "Ende Jacobian-test" << std::endl;
			return error;
		}
		
	protected:
		
		// Blöcke
		omnimobot::InvJacobian invJacobiBlock;
//		omnimobot::I<Vector3> integrator;
		eeros::control::Output<Vector3> data;
		eeros::control::Output<Vector3> dataPhiSteer;

};

int main(int argc, char* argv[]) {
	std::cout << "Start Jacobian-test" << std::endl;
	InvJacobiBlockTest<> tester;
	if (argc == 2) {
		
		return tester.run(argv[1]);
	}
	else {
		std::cout << "illegal number of arguments: " << argc << std::endl;
	}
	return -3;
	
}




