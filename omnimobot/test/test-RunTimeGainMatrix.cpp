#include <eeros/core/PeriodicThread.hpp>

#include <eeros/control/Constant.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/math/Matrix.hpp>

#include <iostream>
#include <unistd.h>

using namespace eeros;
using namespace eeros::math;
using namespace eeros::logger;


static eeros::math::Matrix<6,6,double> matrixGainValue = eeros::math::Matrix<6,6,double>({	5.0,   0.0,  0.0,    0.0,    0.0,     0.0,
																							0.0,   5.0,  0.0,    0.0,    0.0,     0.0,
																							0.0,   0.0,  5.0,    0.0,    0.0,     0.0,
																							0.0,   0.0,  0.0,    5.0,    0.0,     0.0,
																							0.0,   0.0,  0.0,    0.0,    5.0,     0.0,
																							0.0,   0.0,  0.0,    0.0,    0.0,     5.0}).transpose();

class Gaintest : public PeriodicThread {
	public:
		Gaintest(double period) : 
		desired({0.0,0.0,0.0,0.0,0.0,0.0}),
		value({0.0,0.0,0.0,0.0,0.0,0.0}),
		ctr(0),  
		end(false),
		matrixGain(matrixGainValue),
		PeriodicThread(period, 0, true, paused){
			
			matrixGain.getIn().connect(value.getOut());
		}
			
			
		void run() {
			
			// time: 50000*period
			if(ctr == 50000){
				std::cout << "counter true nach: "<< ctr <<"  value 5: "<< matrixGain.getOut().getSignal().getValue()(5)<<std::endl; 
				end = true;
				stop();
			}
			
			desired = value.getOut().getSignal().getValue(); 
			
			// Rampe
			for (int i = 0.0; i < 6; i++) {
				if(desired(i) < 100.0){
					desired(i) = desired(i)+0.00005;
				}
			}

			value.setValue(desired);
			value.run();
			matrixGain.run();
			
			ctr++;
		}
		
		bool end;
		
	private:
		eeros::control::Gain<eeros::math::Vector<6>, eeros::math::Matrix<6,6>> matrixGain;
		eeros::control::Constant<eeros::math::Vector<6>> value;
		eeros::math::Vector<6> desired;
		int ctr;
};



static volatile bool running = true;
void sig_handler(int signum)
{
    running = false;
}



int main(int argc, char *argv[]) 
{
	
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
// 	w.show(~0); // show all messages
 	w.show(0); 
	Logger<LogWriter> log;

	double periodTime = 0.001; // [s] 
	
	log.info() << "creating test thread";
	Gaintest test(periodTime);
	
	sleep(1);
	
	log.info() << "starting test thread";
	test.start();
	
	log.info() << "waiting for test thread";
	
	while(running && !test.end) {
		sleep(1);
		auto &counter = test.counter;
		std::cout << "run mean: " << counter.run.mean << " run max: " << counter.run.max << "  period min: " << counter.period.min<< "  period max: " << counter.period.max  <<std::endl;
		test.counter.reset();
	}
	
	std::cout << "test thread finished"<<std::endl;

    return 0;
}