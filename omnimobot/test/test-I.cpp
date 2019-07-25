#include <eeros/core/Runnable.hpp>
#include <omnimobot/control/block/I.hpp>
#include <cstdlib>
#include <iostream>
#include <fstream>

#include <Utils.hpp>

template <typename T = double>
class IBlockTest {
	public:
		IBlockTest() {
			integrator.getIn().connect(data);
			fileout.open("/home/ntb/work/transit/FuerMatlab/intOut.txt");
		}
		
		int run(const char* filepath) {
			std::ifstream file(filepath);
			std::cout << "run ausfürend" << std::endl;
			if (!file.is_open()) return -2;
			
			int line = 0;
			int error = 0;
			uint64_t timestamp = 0;
			
			while (!file.eof()) {
				line++;
				double in;
				double out;
				
				file >> in; // read input data
				file >> out; // read reference result
				
				if (file.eof()) break;
				
				data.getSignal().setValue(1);
				data.getSignal().setTimestamp(timestamp);
				timestamp += 1000000;
				
				integrator.run();
				
				if(line>=0 and line<=10) {
					std::cout << line<< std::endl;
					std::cout << "\t" << in <<" eingang: " << integrator.getIn().getSignal().getValue() << std::endl;
					std::cout << "\t" << out <<" berechnet voltS1: " << integrator.getOut().getSignal().getValue() << std::endl << std::endl;
					
					
				}
				fileout << out << "\t" << integrator.getOut().getSignal().getValue() << std::endl;

// 				if(!Utils::compareApprox(out, integrator.getOut().getSignal().getValue(), 0.00001)) {
// 					error++;
// 					std::cout << "line " << line << " expecting " << out << " calculated " << integrator.getOut().getSignal().getValue() << std::endl;
// 				}
				
				
			}
			
			
			fileout.close();
			file.close();
			std::cout << "Ende i-test" << std::endl;
			return error;
		}
		
	protected:
		eeros::control::Output<T> data;
		omnimobot::I<T> integrator;
		std::ofstream fileout;
};

int main(int argc, char* argv[]) {
	std::cout << "Start i-test" << std::endl;
	IBlockTest<> tester;
	if (argc == 2) {
		return tester.run(argv[1]);
	}
	else {
		std::cout << "illegal number of arguments" << std::endl;
	}
	return -3;
	
}
