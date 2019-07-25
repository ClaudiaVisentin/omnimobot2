#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <omnimobot/Counter.hpp>
#include <omnimobot/constants.hpp>
#include <omnimobot/control/block/AccTipToPhi.hpp>

// #include <eeros/hal/HAL.hpp>
// #include <eeros/hal/FlinkAnalogIn.hpp>
// #include <eeros/hal/FlinkDigIn.hpp>
// #include <eeros/control/PeripheralInput.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Constant.hpp>

#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/PeriodicThread.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Sum.hpp>

// #define ON_BOAD_FPGA_DEVICE "/dev/flink0"
// #define ON_BOARD_PWM_ID 0
// #define ON_BOARD_WATCHDOG_ID 1
// #define SENSOR_PRINT_ADC_ID 2 // oder 4
// #define ON_BOARD_FQD_ID 5
// #define ON_BOARD_OUT_IO_ID 6
// #define ON_BOARD_IN_IO_ID 5


// #define HOMSENSOR_RAD1 10
// #define HOMSENSOR_RAD2 14
// #define HOMSENSOR_RAD3 12
using namespace omnimobot;
using namespace eeros;
using namespace eeros::math;
using namespace eeros::logger;
using namespace eeros::control;

class Worker : public PeriodicThread {
// class Worker{
public:
	Worker(double time) : 
	source({0.0,0.0}),
	counter(10),
	tmp(0),
	gKpPos(kpTip),
	gKdPos(kdTip),
	prevX(0),

	PeriodicThread(time, 0, true, paused)
	{	
		gKpPos.getIn().connect(source.getOut());
		diffTip.getIn().connect(source.getOut());
		gKdPos.getIn().connect(diffTip.getOut());
		SumPDTip.getIn(0).connect(gKdPos.getOut());
		SumPDTip.getIn(1).connect(gKpPos.getOut());
		block.getInAccTip().connect(SumPDTip.getOut());	
	}
	
	void run() {

		source.run();
		gKpPos.run();
		diffTip.run();
		gKdPos.run();
		SumPDTip.run();
		block.run();
		
		if(sourceVector(0) > -0.02 && prevX == sourceVector(0)+0.01 || sourceVector(0) == 0.02) {
			sourceVector(0) = sourceVector(0)-0.01;
		}
		else if(sourceVector(0) < 0.02 && prevX == sourceVector(0)-0.01 || sourceVector(0) == -0.02) {
			sourceVector(0) = sourceVector(0)+0.01;
		}
		else {
			sourceVector(0) = 0.02;
		}
		
		if(tmp ==500){
			std::cout << "in: " << source.getOut().getSignal().getValue()(0)  << "   out: "<<block.getOutPhiSoll().getSignal().getValue()(0) << "   outDiff: "<<diffTip.getOut().getSignal().getValue()(0)<< std::endl<< std::endl;
// 			std::cout << "   outDiff: "<<diffTip.getOut().getSignal().getValue()(0)<< std::endl<< std::endl;
			counter.count();
			tmp=0;
		}
		source.setValue(sourceVector);
		prevX = source.getOut().getSignal().getValue()(0);
		
		tmp++;
		
		
		
		if (counter.isCountEnd()) {

			stop();
		}
	}

	
	
private:
	
	Counter counter;

	AccTipToPhi block;
	Gain<Vector2,double> gKpPos;
	Gain<Vector2,double> gKdPos;
	Sum<2,Vector2> SumPDTip;
	D<Vector2> diffTip;
	
	Constant<Vector2> source;
	
	Vector2 sourceVector;
	
	double prevX;
	
	int tmp;
	

};





int main() {
	std::cout << "HallSensor Testing " << std::endl<< std::endl;
	
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	
	log.info() << "Initializing hardware";
	


	log.info() << "hardware configuration done";
	
//	Worker worker(0.001);
	Worker worker(dt);
	log.info() << "starting worker thread";
	worker.start();
	log.info() << "waiting for worker thread";
	worker.join(); // Wartet solange bis der Thred fertig
	
	std::cout << "Test finished..." << std::endl;
}