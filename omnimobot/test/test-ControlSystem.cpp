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
#include <omnimobot/control/ControlSystem.hpp>

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Sum.hpp>
#include <omnimobot/constants.hpp>


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




class Controltest : public PeriodicThread {
	public:
		Controltest(double period, FlinkWatchdog& wd, ControlSystem* cs) : 
		controlSys(cs),
		ctr(40000),		// time to end
		ctr2(500),
		counter(0),
		value(0.0),
		wd(wd),

		PeriodicThread(period, 0, true, paused){
			
			HAL& hal = HAL::instance();
			
			pilz = hal.getLogicPeripheralInput("notstop");
			
			controlSys->initHoming();
			
			controlSys->robotControlBlock.homingBlock.swSteer1.switchToInput(0);
			controlSys->robotControlBlock.homingBlock.swSteer2.switchToInput(0);
			controlSys->robotControlBlock.homingBlock.swSteer3.switchToInput(0);
			controlSys->robotControlBlock.homingBlock.swWeel1.switchToInput(0);
			controlSys->robotControlBlock.homingBlock.swWeel2.switchToInput(0);
			controlSys->robotControlBlock.homingBlock.swWeel3.switchToInput(0);
			
			controlSys->robotControlBlock.swHomingOn.switchToInput(0);
		}
			
			
			
		void startControlSystem(){
			controlSys->start();
		}
			
			
		void run() {
			
			
			
			wd.set(true);
			
			counter++;
			
			ctr.count();
			
			if(ctr.isCountEnd()){
				
				log.info() << "counter true nach: "<< counter;
				stop();
			}
/*			
			value = controlSys->joystickConst.getOut().getSignal().getValue()(0); 
			//Rampe
			if(value < 2.0){
				controlSys->joystickConst.setValue({value+1.0,0.0,0.0,0.0});
			}*/


			notStop = pilz->get(); // immer neu!!!!!
			if (notStop != oldStop) {
				std::cout << "########################### stop:  " << notStop << "  #######################" << std::endl;
			}
			oldStop = notStop;
			

		}
		
		virtual ~Controltest(){};
		
	private:

		
		eeros::hal::PeripheralInput<bool>* pilz;


		FlinkWatchdog& wd;
		ControlSystem* controlSys;
		
		Counter ctr;
		Counter ctr2;
		
		int counter;
		double desired;
		double value;
		bool oldStop;
		bool notStop;


};


static volatile bool running = true;
void sig_handler(int signum)
{
    running = false;
}


int main(int argc, char *argv[]) 
{
	
	signal(SIGINT, sig_handler);
	
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	w.show(~0); // show all messages
	//w.show(3); // show first four levels
	Logger<LogWriter> log;
	
	// Start
	log.info() << "Test control started...";
	
	// Initialize hardware
	log.info() << "Initializing hardware";
	
	HAL& hal = HAL::instance();
	
	log.trace() << "  Creating device structure...";
	FlinkDevice onBoard (ON_BOAD_FPGA_DEVICE);
	
	// Encoder
	FlinkFqd enc_0("q0", &onBoard, ON_BOARD_FQD_ID, 0, 6.28318530718 / (4 * 1024*18), 0,true); // 36.75 ist die Getriebe übersetzung Lenkung, 18:antrieb
	FlinkFqd enc_1("q1", &onBoard, ON_BOARD_FQD_ID, 1, 6.28318530718 / (4 * 1024*18), 0,true); // true: delta q wird ausgegeben
	FlinkFqd enc_2("q2", &onBoard, ON_BOARD_FQD_ID, 2, 6.28318530718 / (4 * 1024*18), 0,true);
	FlinkFqd enc_3("q3", &onBoard, ON_BOARD_FQD_ID, 3, 6.28318530718 / (4 * 1024*36.75), 0,true);
	FlinkFqd enc_4("q4", &onBoard, ON_BOARD_FQD_ID, 4, 6.28318530718 / (4 * 1024*36.75), 0,true);
	FlinkFqd enc_5("q5", &onBoard, ON_BOARD_FQD_ID, 5, 6.28318530718 / (4 * 1024*36.75), 0,true);
	
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
	
	// Im Hal dazu fuegen
	hal.addPeripheralInput(&enc_0);
	hal.addPeripheralInput(&enc_1);
	hal.addPeripheralInput(&enc_2);
	hal.addPeripheralInput(&enc_3);
	hal.addPeripheralInput(&enc_4);
	hal.addPeripheralInput(&enc_5);
	
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
	
	hal.addPeripheralInput(&homeSensor1);
	
	hal.addPeripheralInput(&notStop);
	


	
	double periodTime = 0.001; // [s] weniger wie 0.008 s
	
	// craete control system
	log.info() << "create control System";
	ControlSystem controlSys(periodTime);

	sleep(1);
	
	Controltest test(periodTime,watchdog,&controlSys);

	sleep(1);
	
	log.info() << "starting Wd";
	watchdog.reset();
	log.info() << "vor startControl";
	test.startControlSystem();
	log.info() << "starting test thread";
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

	log.trace() << "Test control ended";
    return 0;
}