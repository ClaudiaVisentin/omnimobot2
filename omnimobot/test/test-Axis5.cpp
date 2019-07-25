#include <eeros/hal/HAL.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/FlinkDevice.hpp>
#include <eeros/hal/FlinkFqd.hpp>
#include <eeros/hal/FlinkPwm.hpp>
#include <eeros/hal/FlinkWatchdog.hpp>
#include <eeros/core/PeriodicThread.hpp>
#include <eeros/hal/FlinkDigIn.hpp>
#include <eeros/hal/FlinkDigIn.hpp>
#include <omnimobot/Counter.hpp>

#include <eeros/control/DeMux.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/Saturation.hpp>
#include <omnimobot/control/block/I.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Sum.hpp>
#include <omnimobot/constants.hpp>
#include <omnimobot/control/block/MotorModel.hpp>
#include <omnimobot/control/block/VoltageToPWM.hpp>
#include <omnimobot/control/block/InvJacobian.hpp>

#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/PeripheralOutput.hpp>

#include <types.hpp>
#include <signal.h>
#include <iostream>
#include <unistd.h>

#define ON_BOAD_FPGA_DEVICE "/dev/flink0"

// Subdevices
#define ON_BOARD_PWM_ID 0
#define ON_BOARD_FQD_ID 3
#define ON_BOARD_IN_IO_ID 5
#define ON_BOARD_OUT_IO_ID 4

// Homing ID
#define HOMSENSOR_RAD1 10
#define HOMSENSOR_RAD2 14
#define HOMSENSOR_RAD3 12

#define ON_BOARD_WATCHDOG_ID 1


using namespace eeros;
using namespace omnimobot;
using namespace eeros::logger;
using namespace eeros::hal;




class axis5 : public PeriodicThread {
	public:
		axis5(double period, FlinkWatchdog& wd, FlinkDigIn& sensor) : 
		pwmA5("pwmAaxis5"),
		pwmB5("pwmBaxis5"),
		pwmA4("pwmAaxis4"),
		pwmB4("pwmBaxis4"),
		pwmA3("pwmAaxis3"),
		pwmB3("pwmBaxis3"),
		sensor(sensor),
		value(0.1),
		value4(0.1),
		zero(0.0),
		wd(wd),
		ctr(30000),
		ctr2(15000),
		ctr3(0),
		PeriodicThread(period, 0, true, paused){
			
			HAL& hal = HAL::instance();
			pwmA5.getIn().connect(value4.getOut());
			pwmB5.getIn().connect(zero.getOut());
			
			pwmA4.getIn().connect(value4.getOut());
			pwmB4.getIn().connect(zero.getOut());
			
			pwmA3.getIn().connect(value4.getOut());
			pwmB3.getIn().connect(zero.getOut());
			
// 			pwmA5.getIn().connect(zero.getOut());
// 			pwmB5.getIn().connect(value4.getOut());
// 			
// 			pwmA4.getIn().connect(zero.getOut());
// 			pwmB4.getIn().connect(value4.getOut());
// 			
// 			pwmA3.getIn().connect(zero.getOut());
// 			pwmB3.getIn().connect(value4.getOut());
// 			
			
		}
			
		void run() {
			
			static bool first = true;
// 			
// 			if (first) {
// 				value.run();
// 				value4.run();
// 				zero.run();
// 				pwmA5.run();
// 				pwmB5.run();
// 				pwmA4.run();
// 				pwmB4.run();
// 				first = false;
// 				log.info() << "value sended...";
// 			}
			

			wd.set(true);
			
			value.run();
			value4.run();
			zero.run();
			pwmA5.run();
			pwmB5.run();
			pwmA4.run();
			pwmB4.run();
			pwmA3.run();
			pwmB3.run();

// 			if(sensor.get()) { // Endschalter erreicht
// 				stop();
// 			}
			
			ctr.count();
			ctr3++;
			
			if(ctr.isCountEnd()){
				
				log.info() << "counter true nach: "<< ctr3;
				stop();
			}
			
			ctr2.count();
			if(ctr2.isCountEnd() && first){
				
				
				
				first = false;
			}

		}
		
		
	private:
			
		eeros::control::PeripheralOutput<double> pwmA5;
		eeros::control::PeripheralOutput<double> pwmB5;
		eeros::control::PeripheralOutput<double> pwmA4;
		eeros::control::PeripheralOutput<double> pwmB4;
		eeros::control::PeripheralOutput<double> pwmA3;
		eeros::control::PeripheralOutput<double> pwmB3;
		
		eeros::control::Constant<double> value;
		eeros::control::Constant<double> value4;
		eeros::control::Constant<double> zero;
		
		FlinkWatchdog& wd;
		FlinkDigIn& sensor;
		Counter ctr;
		Counter ctr2;
		int ctr3;	

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
	w.show(~0); // show all messages
	//w.show(3); // show first four levels
	Logger<LogWriter> log;
	
	// Start
	log.info() << "Test axis5 started...";
	
	// Initialize hardware
	log.info() << "Initializing hardware";
	
	HAL& hal = HAL::instance();
	
	log.trace() << "  Creating device structure...";
	FlinkDevice onBoard (ON_BOAD_FPGA_DEVICE);
	
	// PWM
	FlinkPwm pwmAaxis3("pwmAaxis3",&onBoard, 0, 6);
	FlinkPwm pwmBaxis3("pwmBaxis3",&onBoard, 0, 7);
	FlinkPwm pwmAaxis4("pwmAaxis4",&onBoard, 0, 8);
	FlinkPwm pwmBaxis4("pwmBaxis4",&onBoard, 0, 9);
	FlinkPwm pwmAaxis5("pwmAaxis5",&onBoard, 0, 10);
	FlinkPwm pwmBaxis5("pwmBaxis5",&onBoard, 0, 11);
	
	// Frequenz von pwm setzen
	pwmAaxis3.setFrequency(25000);
	pwmBaxis3.setFrequency(25000);
	pwmAaxis4.setFrequency(25000);
	pwmBaxis4.setFrequency(25000);
	pwmAaxis5.setFrequency(25000);
	pwmBaxis5.setFrequency(25000);
	
// 	// Watchdog
	FlinkWatchdog watchdog("onBoardWatchdog", &onBoard, ON_BOARD_WATCHDOG_ID,0.5); // [s]// braucht min. alle 0.009 s
	
	// homing sensor
	FlinkDigIn homeSensor2("Wheel1", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD2); // zum Invertieren letzten Parameter auf true setzen, Zahl ist welcher Sensorbenutzt wird
	
	// Im Hal dazu fuegen

	hal.addPeripheralOutput(&pwmAaxis5);
	hal.addPeripheralOutput(&pwmBaxis5);
	hal.addPeripheralOutput(&pwmAaxis4);
	hal.addPeripheralOutput(&pwmBaxis4);
	hal.addPeripheralOutput(&pwmAaxis3);
	hal.addPeripheralOutput(&pwmBaxis3);

	hal.addPeripheralOutput(&watchdog);
	
	hal.addPeripheralInput(&homeSensor2);

	
	//PWM A
	std::cout << "Setup PWM " << std::endl;

	double periodTime = 0.001; // [s] 
	
	log.info() << "creating test thread";
	axis5 test(periodTime,watchdog,homeSensor2);
	
	sleep(1);
	
	log.info() << "starting test thread";
	watchdog.reset();
	test.start();
// 	sleep(1);
	log.info() << "waiting for test thread";
	
	while(running) {
		sleep(1);
		//auto &counter = safetySystem.counter;
// 		auto &counter = test.counter;
// 		log.fatal() << "mean: " << counter.run.mean << " max: " << counter.run.max;
	}
	
	sleep(1);
// 	test.join(); // Wartet solange bis der Thred fertig
	
	log.info() << "test thread finished";
	
	// pwm zurücksetzen
	pwmBaxis5.setDutyCycle(0.0);
	pwmAaxis5.setDutyCycle(0.0);
	pwmBaxis4.setDutyCycle(0.0);
	pwmAaxis4.setDutyCycle(0.0);
	pwmBaxis3.setDutyCycle(0.0);
	pwmAaxis3.setDutyCycle(0.0);

	log.trace() << "Test drive ended";
    return 0;
}