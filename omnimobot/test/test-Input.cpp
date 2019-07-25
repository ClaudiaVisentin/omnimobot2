#include <eeros/hal/FlinkDigIn.hpp>

#define ON_BOAD_FPGA_DEVICE "/dev/flink0"
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/HAL.hpp>
#include <signal.h>
#include <iostream>
#include <unistd.h>
#include <eeros/hal/FlinkPwm.hpp>

// Subdevices
#define ON_BOARD_IN_IO_ID 5

// Homing ID
#define HOMSENSOR_RAD1 10
#define HOMSENSOR_RAD2 14

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::hal;

int main(int argc, char *argv[])
{
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	w.show(~0); // show all messages
	//w.show(3); // show first four levels
	Logger<LogWriter> log;
	
	// Start
	log.info() << "Application Homingsensortest started...";
	
	// Initialize hardware
	log.info() << "Initializing hardware";
	HAL& hal = HAL::instance();
	
	log.trace() << "  Creating device structure...";
	FlinkDevice onBoard (ON_BOAD_FPGA_DEVICE);
	
	
	
	// Homingsensoren
	FlinkDigIn homeSensor1("Rad1", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD1);
	FlinkDigIn homeSensor2("Rad2", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD2);
	
	FlinkPwm pwmAaxis3("pwmAaxis3",&onBoard, 0, 6);
	FlinkPwm pwmBaxis3("pwmBaxis3",&onBoard, 0, 7);
	
	bool home1 = false;
	bool home1old = false;
	
	pwmAaxis3.setFrequency(25000);
	pwmBaxis3.setFrequency(25000);
	
	pwmAaxis3.setDutyCycle(0.0);
	pwmBaxis3.setDutyCycle(0.1);
	
	
	while(!homeSensor2.get()){
		
		home1 = homeSensor1.get();
		
		if(home1 != home1old){
			log.trace() << " Sensor impuls " << home1;
		}
		
		home1old = home1;
		
	}
	
	log.trace() << "Test sensor ended";
}