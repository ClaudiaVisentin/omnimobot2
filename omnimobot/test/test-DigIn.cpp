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
#include <eeros/hal/PeripheralInput.hpp>
#include <omnimobot/Counter.hpp>

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



class SPItest : public PeriodicThread {
	public:
		SPItest(double period,FlinkPwm& pwmAax0,FlinkPwm& pwmAax1,FlinkPwm& pwmAax2,
		FlinkPwm& pwmAax3,FlinkPwm& pwmAax4,FlinkPwm& pwmAax5,FlinkPwm& pwmBax0,FlinkPwm& pwmBax1,FlinkPwm& pwmBax2,FlinkPwm& pwmBax3,FlinkPwm& pwmBax4,
		FlinkPwm& pwmBax5,FlinkDigIn& sensor1 ,FlinkDigIn& sensor2,FlinkDigIn& sensor3,FlinkDigIn& notStop,FlinkWatchdog& wd) : 
		pwmAax0(pwmAax0),
		pwmAax1(pwmAax1),
		pwmAax2(pwmAax2),
		pwmAax3(pwmAax3),
		pwmAax4(pwmAax4),
		pwmAax5(pwmAax5),
		pwmBax0(pwmBax0),
		pwmBax1(pwmBax1),
		pwmBax2(pwmBax2),
		pwmBax3(pwmBax3),
		pwmBax4(pwmBax4),
		pwmBax5(pwmBax5),
		value(0.0),
// 		sensor1(sensor1),
// 		sensor2(sensor2),
// 		sensor3(sensor3),
// 		notStop(notStop),
		counter(0),
		wd(wd),
		ctr(50000),
		PeriodicThread(period, 0, true, paused){
			
			HAL& hal = HAL::instance();
			
			homeSensorWheel1 = hal.getLogicPeripheralInput("Wheel1");
			homeSensorWheel2 = hal.getLogicPeripheralInput("Wheel2");
			homeSensorWheel3 = hal.getLogicPeripheralInput("Wheel3");
			
			pilz = hal.getLogicPeripheralInput("notstop");

		}
			
		void run() {
			wd.set(true);
			
			counter++;
			
			ctr.count();
			
			if(ctr.isCountEnd()){
				
				log.info() << "counter true nach: "<< counter;
				stop();
			}
			
			// Rampe
			if(value < 0.15){
				value = value + 0.00001;
			}
			
			pwmAax0.setDutyCycle(value);
			pwmAax1.setDutyCycle(value);
			pwmAax2.setDutyCycle(value);
			pwmAax3.setDutyCycle(value);
			pwmAax4.setDutyCycle(value);
			pwmAax5.setDutyCycle(value);
			
			pwmBax0.setDutyCycle(0.0);
			pwmBax1.setDutyCycle(0.0);
			pwmBax2.setDutyCycle(0.0);
			pwmBax3.setDutyCycle(0.0);
			pwmBax4.setDutyCycle(0.0);
			pwmBax5.setDutyCycle(0.0);
			
			
			                                             
// 			static bool old1 = false;
// 			bool rad1 = homeSensorWheel1->get();
// 			if (rad1 != old1) {
// 				std::cout << "########################### rad1:  " << rad1 << "  #######################" << std::endl;
// 			}
// 			old1 = rad1;
			
// 			static bool old2 = false;
// 			bool rad2 = homeSensorWheel2->get();
// 			if (rad2 != old2) {
// 				std::cout << "########################### rad2:  " << rad2 << "  #######################" << std::endl;
// 			}
// 			old2 = rad2;
// 			
// 			static bool old3 = false;
// 			bool rad3 = homeSensorWheel3->get();
// 			if (rad3 != old3) {
// 				std::cout << "########################### rad3:  " << rad3 << "  #######################" << std::endl;
// 			}
// 			old3 = rad3;
			
			
			static bool oldStop = false;
			bool stop = pilz->get();
			if (stop != oldStop) {
				std::cout << "########################### stop:  " << stop << "  #######################" << std::endl;
			}
			oldStop = stop;
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
// 			static bool old1 = false;
// 			bool rad1 = sensor1.get();
// 			if (rad1 != old1) {
// 				std::cout << "########################### rad1:  " << rad1 << "  #######################" << std::endl;
// 			}
// 			old1 = rad1;
			
// 			static bool old2 = false;
// 			bool rad2 = sensor2.get();
// 			if (rad2 != old2) {
// 				std::cout << "########################### rad2:  " << rad2 << "  #######################" << std::endl;
// 			}
// 			old2 = rad2;
// 			
// 			static bool old3 = false;
// 			bool rad3 = sensor3.get();
// 			if (rad3 != old3) {
// 				std::cout << "########################### rad3:  " << rad3 << "  #######################" << std::endl;
// 			}
// 			old3 = rad3;
// 			
// 			
// 			static bool oldStop = false;
// 			bool stop = notStop.get();
// 			if (stop != oldStop) {
// 				std::cout << "########################### stop:  " << stop << "  #######################" << std::endl;
// 			}
// 			oldStop = stop;
// 			
			
		}
		
		
		
		
		
	private:
			

			
		FlinkPwm& pwmAax0;	
		FlinkPwm& pwmAax1;	
		FlinkPwm& pwmAax2;	
		FlinkPwm& pwmAax3;	
		FlinkPwm& pwmAax4;	
		FlinkPwm& pwmAax5;	
		FlinkPwm& pwmBax0;	
		FlinkPwm& pwmBax1;	
		FlinkPwm& pwmBax2;	
		FlinkPwm& pwmBax3;	
		FlinkPwm& pwmBax4;	
		FlinkPwm& pwmBax5;	
		
		FlinkWatchdog& wd;
		
		eeros::hal::PeripheralInput<bool>* homeSensorWheel1;
		eeros::hal::PeripheralInput<bool>* homeSensorWheel2;
		eeros::hal::PeripheralInput<bool>* homeSensorWheel3;
		
		eeros::hal::PeripheralInput<bool>* pilz;	
		
// 		FlinkDigIn& sensor1;
// 		FlinkDigIn& sensor2;
// 		FlinkDigIn& sensor3;
		
// 		FlinkDigIn& notStop;
		
		Counter ctr;
		
		double angle;	
		double value;
		int counter;	
		int i;	

};



int main(int argc, char *argv[]) 
{
	
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	w.show(~0); // show all messages
	//w.show(3); // show first four levels
	Logger<LogWriter> log;
	
	// Start
	log.info() << "Test Input started...";
	
	// Initialize hardware
	log.info() << "Initializing hardware";
	
	HAL& hal = HAL::instance();
	
	log.trace() << "  Creating device structure...";
	FlinkDevice onBoard (ON_BOAD_FPGA_DEVICE);
	
	
	// PWM
	FlinkPwm pwmAaxis0("pwmAaxis0",&onBoard, 0, 0);
	FlinkPwm pwmAaxis1("pwmAaxis1",&onBoard, 0, 2);
	FlinkPwm pwmAaxis2("pwmAaxis2",&onBoard, 0, 4);
	FlinkPwm pwmAaxis3("pwmAaxis3",&onBoard, 0, 6);
	FlinkPwm pwmAaxis4("pwmAaxis4",&onBoard, 0, 8);
	FlinkPwm pwmAaxis5("pwmAaxis5",&onBoard, 0, 10);
	FlinkPwm pwmBaxis0("pwmBaxis0",&onBoard, 0, 1);
	FlinkPwm pwmBaxis1("pwmBaxis1",&onBoard, 0, 3);
	FlinkPwm pwmBaxis2("pwmBaxis2",&onBoard, 0, 5);
	FlinkPwm pwmBaxis3("pwmBaxis3",&onBoard, 0, 7);
	FlinkPwm pwmBaxis4("pwmBaxis4",&onBoard, 0, 9);
	FlinkPwm pwmBaxis5("pwmBaxis5",&onBoard, 0, 11);
	
	// Frequenz von pwm setzen
	pwmAaxis0.setFrequency(25000);
	pwmAaxis1.setFrequency(25000);
	pwmAaxis2.setFrequency(25000);
	pwmAaxis3.setFrequency(25000);
	pwmAaxis4.setFrequency(25000);
	pwmAaxis5.setFrequency(25000);
	pwmBaxis0.setFrequency(25000);
	pwmBaxis1.setFrequency(25000);
	pwmBaxis2.setFrequency(25000);
	pwmBaxis3.setFrequency(25000);
	pwmBaxis4.setFrequency(25000);
	pwmBaxis5.setFrequency(25000);
	
// 	// Watchdog
	FlinkWatchdog watchdog("onBoardWatchdog", &onBoard, ON_BOARD_WATCHDOG_ID,0.008); // [s]// braucht min. alle 0.009 s
	
	// Notstop
	FlinkDigIn notStop("notstop", &onBoard, ON_BOARD_IN_IO_ID, 13, true); 
	
	// homing sensor
	FlinkDigIn homeSensor1("Wheel1", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD1); // zum Invertieren letzten Parameter auf true setzen, Zahl ist welcher Sensorbenutzt wird
	FlinkDigIn homeSensor2("Wheel2", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD2); // zum Invertieren letzten Parameter auf true setzen, Zahl ist welcher Sensorbenutzt wird
	FlinkDigIn homeSensor3("Wheel3", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD3); // zum Invertieren letzten Parameter auf true setzen, Zahl ist welcher Sensorbenutzt wird
	
	// Im Hal dazu fuegen

	
	hal.addPeripheralOutput(&pwmAaxis0);
	hal.addPeripheralOutput(&pwmAaxis1);
	hal.addPeripheralOutput(&pwmAaxis2);
	hal.addPeripheralOutput(&pwmAaxis3);
	hal.addPeripheralOutput(&pwmAaxis4);
	hal.addPeripheralOutput(&pwmAaxis5);
	hal.addPeripheralOutput(&pwmBaxis0);
	hal.addPeripheralOutput(&pwmBaxis1);
	hal.addPeripheralOutput(&pwmBaxis2);
	hal.addPeripheralOutput(&pwmBaxis3);
	hal.addPeripheralOutput(&pwmBaxis4);
	hal.addPeripheralOutput(&pwmBaxis5);

	hal.addPeripheralOutput(&watchdog);
	
	hal.addPeripheralInput(&notStop);
	
	hal.addPeripheralInput(&homeSensor1);
	hal.addPeripheralInput(&homeSensor2);
	hal.addPeripheralInput(&homeSensor3);
	
	//PWM A
	std::cout << "Setup PWM " << std::endl;
	


	
	double periodTime = 0.001; // [s] weniger wie 0.008 s
	
	log.info() << "creating test thread";
	SPItest test(periodTime,pwmAaxis0,pwmAaxis1,pwmAaxis2,pwmAaxis3,pwmAaxis4,pwmAaxis5,pwmBaxis0,pwmBaxis1,pwmBaxis2,pwmBaxis3,pwmBaxis4,pwmBaxis5, homeSensor1, homeSensor2, homeSensor3,notStop, watchdog	);
	
	sleep(1);
	
	log.info() << "starting test thread";
	watchdog.reset();
	test.start();
	
	log.info() << "waiting for test thread";
	test.join(); // Wartet solange bis der Thred fertig
	
	log.info() << "test thread finished";
	
	// pwm zurücksetzen
	pwmBaxis0.setDutyCycle(0.0);
	pwmBaxis1.setDutyCycle(0.0);
	pwmBaxis2.setDutyCycle(0.0);
	pwmBaxis3.setDutyCycle(0.0);
	pwmBaxis4.setDutyCycle(0.0);
	pwmBaxis5.setDutyCycle(0.0);
	
	pwmAaxis0.setDutyCycle(0.0);
	pwmAaxis1.setDutyCycle(0.0);
	pwmAaxis2.setDutyCycle(0.0);
	pwmAaxis3.setDutyCycle(0.0);
	pwmAaxis4.setDutyCycle(0.0);
	pwmAaxis5.setDutyCycle(0.0);

	log.trace() << "Test Input ended";
    return 0;
}