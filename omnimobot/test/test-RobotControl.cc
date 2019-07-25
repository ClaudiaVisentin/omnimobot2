#include <eeros/core/Runnable.hpp>
#include <omnimobot/control/block/RobotControl.hpp>
#include <ostream>
#include <iostream>
#include <fstream>
#include <eeros/logger/StreamLogWriter.hpp>
#include <stdlib.h>
#include <cstdlib>

#include <Utils.hpp>
#include <unistd.h>

using namespace eeros;
using namespace eeros::math;
using namespace eeros::logger;
using namespace std;


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



class RobotControlTest {
	public:
		RobotControlTest( Matrix<6,6> i, Matrix<6,6> invGear, Matrix<6,6> inertia, Matrix<6,1> torqueLim, Matrix<6,1> speedLim) :
		robotControl( i, invGear, inertia,  torqueLim, speedLim )
		{
			robotControl.swHomingOn.switchToInput(0);
			robotControl.homingBlock.swSteer1.switchToInput(0);
			robotControl.homingBlock.swSteer2.switchToInput(0);
			robotControl.homingBlock.swSteer3.switchToInput(0);
			robotControl.homingBlock.swWeel1.switchToInput(0);
			robotControl.homingBlock.swWeel2.switchToInput(0);
			robotControl.homingBlock.swWeel3.switchToInput(0);
			// Verknüpfung RobotControl
			
			robotControl.getInPosActualQ().connect(pos);
			robotControl.getInSpeedActualQ().connect(speed);
			robotControl.getInVeloGlobal().connect(vLokal);
		}
		
		int run(const char* filepath) {
			std::ifstream file(filepath);
			std::cout << "run ausfürend" << std::endl;
			if (!file.is_open()) return -2;
			

			
			int line = 0;
			int error = 0;
			uint64_t timestamp = 0;
			bool first = true;
//			double lastPhiSteer;
				
// 				std::ofstream plot;
// 				plot.open("/home/ntb/work/transit/FuerMatlab/inv_jacobian_test_daten_plot.txt");
// 			
			while (!file.eof()) {
				line++;
				
				
// 				if (first){
// 					lastPhiSteer = 0;
// 				}
				
				Vector3 vLIn; // vx,vy,d_theta
				Vector6 posIn; //pWeel1,pWeel2,pWeel3,pSteer1,pSteer2,pSteer3
				Vector6 speedIn; // omegaW1,omegaW2,omegaW3,omegaS1,omegaS2,omegaS3
				Vector3 beta; // beta1,beta2,beta3
				Vector6 voltageOut; // voltW1,voltW2,voltW3,vlotS1,vlotS2,vlotS3

// //				Vector3 phiSteer; // Ausgang des Integrators von Matlab
// 				phiSteer(0) = 0;
// 				phiSteer(1) = lastPhiSteer;
// 				phiSteer(2) = 0;
				
				file >> vLIn(0); // read input data vx
				file >> vLIn(1); // read input data vy
				file >> vLIn(2); // read input data d_theta
				file >> posIn(0); 
				file >> posIn(1); 
				file >> posIn(2); 
				file >> posIn(3); 
				file >> posIn(4); 
				file >> posIn(5); 
				file >> speedIn(0); 
				file >> speedIn(1); 
				file >> speedIn(2); 
				file >> speedIn(3); 
				file >> speedIn(4); 
				file >> speedIn(5); 
				file >> beta(0); 
				file >> beta(1); 
				file >> beta(2); 
				file >> voltageOut(0); 
				file >> voltageOut(1); 
				file >> voltageOut(2); 
				file >> voltageOut(3); 
				file >> voltageOut(4); 
				file >> voltageOut(5); 
				
//				file >> lastPhiSteer; // phi_LR_rad2
				
				if (file.eof()) break;
				
				vLokal.getSignal().setValue(vLIn);
				vLokal.getSignal().setTimestamp(timestamp);
				
				pos.getSignal().setValue(posIn);
				pos.getSignal().setTimestamp(timestamp);
				
				speed.getSignal().setValue(speedIn);
				speed.getSignal().setTimestamp(timestamp);
				
				timestamp += 1000000;
				
				robotControl.run();

//				Vector3 phi_LR = integrator.getOut().getSignal().getValue();
			
				Vector6 volt = robotControl.getOutVoltage().getSignal().getValue();
				
				if(line>=0 and line<=10) {
					std::cout << line;
					std::cout << "\t" << vLIn(0) << "\t" << vLIn(1) << "\t" <<  vLIn(2) << std::endl;
// 					std::cout << "\t" << "IntegratorinvJ in: " << robotControl.integratorInvJacobi.getIn().getSignal().getValue()(2) << std::endl;
// 					std::cout << "\t" << "IntegratorinvJ out: " << robotControl.integratorInvJacobi.getOut().getSignal().getValue()(2) << std::endl;
					std::cout << "\t" << "Integrator out: " << robotControl.integrator.getOut().getSignal().getValue()(2) << std::endl;
					std::cout << "\t" << "JacobianOut: " << robotControl.invjacobi.getOutOmega().getSignal().getValue()(2)<<std::endl;
					std::cout << "\t" << "SwitchOut: " << robotControl.swHomingOn.getOut().getSignal().getValue()(2) <<std::endl;//.getOut().getSignal().getValue()(1) <<std::endl;
// 					std::cout << "\t" << "SumOut: " << robotControl.sum2.getOut().getSignal().getValue()(2) <<std::endl;//.getOut().getSignal().getValue()(1) <<std::endl;
// 					std::cout << "\t" << "sum1_Out: " << robotControl.sum1.getOut().getSignal().getValue()(2) <<std::endl;//.getOut().getSignal().getValue()(1) <<std::endl;
					std::cout << "\t" << "HomingblockIn: " << robotControl.homingBlock.getInTorque().getSignal().getValue()(2) <<std::endl;//.getOut().getSignal().getValue()(1) <<std::endl;
// 					std::cout << "\t" << "Homingblockout: " << robotControl.homingBlock.getHomingTorqueOutput().getSignal().getValue()(1) << std::endl;
// 					std::cout << "\t" << "gainKpOut: " << robotControl.kpGain.getOut().getSignal().getValue()(2) <<std::endl;//.getOut().getSignal().getValue()(1) <<std::endl;
// 					std::cout << "\t" << "MotormodelIn: " << robotControl.motModel.getTorqueInput().getSignal().getValue()(3) << std::endl;
					std::cout << "\t" << "q: " << robotControl.getInPosActualQ().getSignal().getValue()(2) <<std::endl;//.getOut().getSignal().getValue()(1) <<std::endl;
// 					std::cout << "\t" << "Motormodel: " << robotControl.motModel.invGearGain.getOut().getSignal().getValue()(1) << std::endl;
// 					std::cout << "\t" << voltageOut(0) <<" berechnet voltW1: " << volt(0) << std::endl;
// 					std::cout << "\t" << voltageOut(1) <<" berechnet voltW2: " << volt(1) << std::endl;
// 					std::cout << "\t" << voltageOut(2) <<" berechnet voltW3: " << volt(2) << std::endl;
// 					std::cout << "\t" << voltageOut(3) <<" berechnet voltS1: " << volt(3) << std::endl;
// 					std::cout << "\t" << voltageOut(4) <<" berechnet voltS2: " << volt(4) << std::endl;
					std::cout << "\t" << voltageOut(5) <<" berechnet voltS3: " << volt(5) << std::endl << std::endl;
					
				}
// 				
// 				l
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

// 				if(!Utils::compareApprox(out(0),omega(1) , 0.000001)) {
// 					std::cout << "line " << line << " expecting " << out(0) << " calculated " << omega(1) << std::endl;
// 					error = 1;
// 					
// 				}
// 				if(!Utils::compareApprox(out(1),omega(4) , 0.000001)) {
// 					std::cout << "line " << line << " expecting " << out(1) << " calculated " << omega(4) << std::endl;
// 					error = 2;
//
// 				}

				first = false;
			}
			
			file.close();
//			plot.close();
			std::cout << "Ende RobotControl-test" << std::endl;
			return error;
		}
		
	protected:
		
		// Blöcke
		omnimobot::RobotControl robotControl;
		eeros::control::Output<Vector3> vLokal;
		eeros::control::Output<Vector6> pos;
		eeros::control::Output<Vector6> speed;

};

int main(int argc, char* argv[]) {
	std::cout << "Start RobotControl-test" << std::endl;
	// Create and initialize logger
//	StreamLogWriter w(std::cout);
// 	Logger<LogWriter>::setDefaultWriter(&w);
// 	Logger<LogWriter> log;
	
// 	double km = 0.0302;
// 
// 	double R = 0.299;
// 	
// 	double kv = 500.0;
// 	
// 	double kp = 2.551020408163266e+02;

			
	Matrix<6,6> i;
	i << 	18.0, 0.0,  0.0, 0.0,    0.0,    0.0,
			0.0, 18.0,  0.0, 0.0,    0.0,    0.0,
			0.0,  0.0, 18.0, 0.0,    0.0,    0.0,
			0.0,  0.0,  0.0, 36.75,  0.0,    0.0,
			0.0,  0.0,  0.0, 0.0,    36.75,  0.0,
			0.0,  0.0,  0.0, 0.0,    0.0,    36.75;
			
	Matrix<6,6> invGear;
	invGear << 	1.0/18.0, 0.0,      0.0,      0.0,       0.0,      0.0,
			     0.0,     1.0/18.0, 0.0,      0.0,       0.0,      0.0,
			     0.0,     0.0,      1.0/18.0, 0.0,       0.0,      0.0,
			     0.0,     0.0,      0.0,      1.0/36.75, 0.0,      0.0,
			     0.0,     0.0,      0.0,      0.0,       1.0/36.75,0.0,
			     0.0,     0.0,      0.0,      0.0,       0.0,      1.0/36.75;
				 
	std::cout << "invGear: " << invGear(1,1) << std::endl;

			
	Matrix<6,6> inertia;
	inertia << 	0.034279,   0.0,       0.0,       0.0,       0.0,       0.0,
				0.0,       0.034279,   0.0,       0.0,       0.0,       0.0,
				0.0,       0.0,       0.034279,   0.0,       0.0,       0.0,
				0.0,       0.0,       0.0,       0.0668063,   0.0,       0.0,
				0.0,       0.0,       0.0,       0.0,       0.0668063,   0.0,
				0.0,       0.0,       0.0,       0.0,       0.0,       0.0668063;

				
	Matrix<6,1> torqueLimit;
	torqueLimit << 	0.53,
					0.53,
					0.53,
					0.53,
					0.53,
					0.53;
			
	Matrix<6,1> speedLimit;
	speedLimit << 41.8,//20.0, // [rad/s] (see doku)
				  41.8,//20.0,
				  41.8,//20.0,
				  20.5,//10.36,
				  20.5,//10.36,
				  20.5;//10.36;
					
// 	Matrix<6,1> homingspeed;          s
// 	homingspeed << 	0.2,
// 					0.2,
// 					0.2,
// 					0.2,
// 					0.2,
// 					0.2;
					
// 	Vector3 offset;
// 	offset.zero();

	
	RobotControlTest tester(i,invGear,inertia,torqueLimit,speedLimit);
	if (argc == 2) {
		
		return tester.run(argv[1]);
	}
	else {
		std::cout << "illegal number of arguments: " << argc << std::endl;
	}
	return -3;
	
}
