#include <eeros/hal/HAL.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/FlinkDevice.hpp>
#include <eeros/hal/FlinkFqd.hpp>
#include <eeros/hal/FlinkPwm.hpp>
#include <eeros/hal/FlinkWatchdog.hpp>
#include <eeros/core/PeriodicThread.hpp>
#include <eeros/hal/FlinkDigIn.hpp>
#include <eeros/hal/PeripheralInput.hpp>
#include <omnimobot/Counter.hpp>
#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <omnimobot/control/ControlSystem.hpp>
#include <eeros/hal/FlinkAnalogIn.hpp>

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
#define SENSOR_PRINT_ADC_ID 2 // oder 4


using namespace eeros;
using namespace omnimobot;
using namespace eeros::logger;
using namespace eeros::hal;
using namespace eeros::safety;


namespace event
{
// Name all event
	enum
	{
		InitDone = 11,
		approvalOn = 12,
		stopDrive = 13,

	};
}

namespace level
{
	// Define all possible level
	enum
	{

		initializing = 110,
		powerOn = 220,
		drive = 230,

	};
}

class lowSafetyProperties : public eeros::safety::SafetyProperties {
	public:
		lowSafetyProperties(ControlSystem* cs, FlinkWatchdog& wd) : 
		controlSys(cs),
		watchdog(wd),
		oldStop(false),
		stop(false),
		desired(0.0),
		counter(1000)
		{
			
			if(controlSys == nullptr) {
				throw -325; // TODO
			}
			
			
			encOffset.zero();
			
			HAL& hal = HAL::instance();
			
			homeSensorWheel1 = hal.getLogicPeripheralInput("Wheel1");
			homeSensorWheel2 = hal.getLogicPeripheralInput("Wheel2");
			homeSensorWheel3 = hal.getLogicPeripheralInput("Wheel3");
			
			pilz = hal.getLogicPeripheralInput("notstop");
			
			approval = hal.getLogicPeripheralInput("approval");
			
			emergencyStop = hal.getLogicPeripheralInput("emergencyStop");
			
			criticalOutputs = { &watchdog }; 
			
			
			levels =
			{
				       
				{ ::level::initializing,      "software is initializing",               				 },
				{ ::level::powerOn,           "Power on, controller on, homing starting", 				 },
				{ ::level::drive,           "drive with ramp", 				 }
				
			};
			
			// ############ Add events to the levels ############
			entryLevel = ::level::initializing; // Das erste Level nach dem Start
			
			level( ::level::initializing      ).addEvent( event::InitDone,              ::level::powerOn,           eeros::safety::kPublicEvent);
			level( ::level::powerOn           ).addEvent( event::approvalOn,            ::level::drive,           eeros::safety::kPublicEvent);
			level( ::level::drive           ).addEvent( event::stopDrive,              ::level::initializing,           eeros::safety::kPublicEvent);
			
			// ############ Define output states and events for all levels ############
			level(::level::initializing        ).setOutputActions({  set(watchdog, true) });
			level(::level::powerOn             ).setOutputActions({  set(watchdog, true) });
			level(::level::drive               ).setOutputActions({  set(watchdog, true) });
			
			
			
			// Define and add level functions
			level(::level::initializing).setLevelAction([&](SafetyContext* privateContext) {
		
				static bool first = true; 
				if(first == true) {
					watchdog.reset();
					first = false;
				}

				// wait of the approval
				if(!pilz->get()) {  
									
					privateContext->triggerEvent(event::InitDone);
				}
			});
			
			
			
			level(::level::powerOn).setLevelAction([&](SafetyContext* privateContext) {
				// dedect notstop
				stop = pilz->get(); // immer neu Fehler!!!!!
				if (stop != oldStop) {
					std::cout << "########################### stop:  " << stop << "  #######################" << std::endl;
				}
				oldStop = stop;
				
				if(approval->get()){
					controlSys->initHoming();// Muss der Reset taster sein
					
							// Homing Switch
					controlSys->robotControlBlock.homingBlock.swSteer1.switchToInput(0);
					controlSys->robotControlBlock.homingBlock.swSteer2.switchToInput(0);
					controlSys->robotControlBlock.homingBlock.swSteer3.switchToInput(0);
					controlSys->robotControlBlock.homingBlock.swWeel1.switchToInput(0);
					controlSys->robotControlBlock.homingBlock.swWeel2.switchToInput(0);
					controlSys->robotControlBlock.homingBlock.swWeel3.switchToInput(0);
					
					controlSys->robotControlBlock.swHomingOn.switchToInput(0);
					
					
					static bool first2 = true; 
					if(first2 == true) {
						controlSys->start(); // Fehler	!!!!!!!!
						first2 = false;
					}
					 
					
					privateContext->triggerEvent(event::approvalOn);
				}
			});
			
			
			
			level(::level::drive).setLevelAction([&](SafetyContext* privateContext) {
				// dedect notstop
				stop = pilz->get(); // immer neu!!!!!
				if (stop != oldStop) {
					std::cout << "########################### stop:  " << stop << "  #######################" << std::endl;
				}
				oldStop = stop;
				
// 				desired = controlSys->joystickConst.getOut().getSignal().getValue()(0); 
// 				// Rampe
// 				if(desired < 2.0){
// 					controlSys->joystickConst.setValue({desired+1.0, 0.0, 0.0, 0.0});
// 				}

				if(emergencyStop->get()){
					controlSys->allAxisStopped();
					privateContext->triggerEvent(event::stopDrive);
				}
			});

		}
		
		virtual ~lowSafetyProperties(){};

	private:

		ControlSystem* controlSys;
		FlinkWatchdog& watchdog;
		
		eeros::hal::PeripheralInput<bool>* homeSensorWheel1;
		eeros::hal::PeripheralInput<bool>* homeSensorWheel2;
		eeros::hal::PeripheralInput<bool>* homeSensorWheel3;
		
		eeros::hal::PeripheralInput<bool>* emergencyStop; 		// um emergency zu reseten
		eeros::hal::PeripheralInput<bool>* approval;			// Taster
		
		eeros::hal::PeripheralInput<bool>* pilz;	
		
		eeros::math::Matrix<6,1> encOffset;
		Counter counter;
		
		double desired;
		bool oldStop;
		bool stop;
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
	log.info() << "Test with Safty started...";
	
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
	
	
		// HMI
	FlinkDigIn taster1("approval", &onBoard, ON_BOARD_IN_IO_ID, 2); // Anhängen und Kontrollieren
	FlinkDigIn taster2("emergency", &onBoard, ON_BOARD_IN_IO_ID, 0); // Noch mit Piltz verknüpfen
	FlinkDigIn taster3("emergencyStop", &onBoard, ON_BOARD_IN_IO_ID, 6); // Anhängen und Kontrollieren
	
	// Hallsensoren
	FlinkAnalogIn hall0("hall_0", &onBoard, SENSOR_PRINT_ADC_ID,0);
	FlinkAnalogIn hall1("hall_1", &onBoard, SENSOR_PRINT_ADC_ID,1);
	FlinkAnalogIn hall2("hall_2", &onBoard, SENSOR_PRINT_ADC_ID,2);
	FlinkAnalogIn hall3("hall_3", &onBoard, SENSOR_PRINT_ADC_ID,3);
	
	
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
	FlinkWatchdog watchdog("onBoardWatchdog", &onBoard, ON_BOARD_WATCHDOG_ID,0.5); // [s]// braucht min. alle 0.009 s
	
	// Notstop
	FlinkDigIn notStop("notstop", &onBoard, ON_BOARD_IN_IO_ID, 13, true); 
	
	// homing sensor
	FlinkDigIn homeSensor1("Wheel1", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD1); // zum Invertieren letzten Parameter auf true setzen, Zahl ist welcher Sensorbenutzt wird
	FlinkDigIn homeSensor2("Wheel2", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD2); // zum Invertieren letzten Parameter auf true setzen, Zahl ist welcher Sensorbenutzt wird
	FlinkDigIn homeSensor3("Wheel3", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD3); // zum Invertieren letzten Parameter auf true setzen, Zahl ist welcher Sensorbenutzt wird
	
	// Im Hal dazu fuegen

	hal.addPeripheralInput(&hall0); 
	hal.addPeripheralInput(&hall1);
	hal.addPeripheralInput(&hall2);
	hal.addPeripheralInput(&hall3);
	
	
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
	
	hal.addPeripheralInput(&taster1);
	hal.addPeripheralInput(&taster2);
	hal.addPeripheralInput(&taster3);
	hal.addPeripheralInput(&notStop);
	
	hal.addPeripheralInput(&homeSensor1);
	hal.addPeripheralInput(&homeSensor2);
	hal.addPeripheralInput(&homeSensor3);
	


	
	double periodTime = 0.001; // [s] weniger wie 0.008 s
	
	// craete control system
	log.info() << "create control System";
	ControlSystem controlSys(periodTime);
	
	
	log.info() << "create safty System";
	lowSafetyProperties properties(&controlSys, watchdog);
	SafetySystem safetySystem(properties, periodTime); 
	
	while(running) sleep(1);
	
	

	
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

	log.trace() << "Test Input with Safty ended";
    return 0;
}