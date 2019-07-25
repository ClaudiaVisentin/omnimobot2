#include <omnimobot/control/ControlSystemWithJacobi.hpp>
#include <omnimobot/safety/SafetyWithControlJacobiProb.hpp>
#include <omnimobot/constants.hpp>

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/FlinkDevice.hpp>
#include <eeros/hal/FlinkDigIn.hpp>
#include <eeros/hal/FlinkDigOut.hpp>
#include <eeros/hal/FlinkFqd.hpp>
#include <eeros/hal/FlinkPwm.hpp>
#include <eeros/hal/FlinkWatchdog.hpp>
#include <eeros/hal/FlinkAnalogIn.hpp>

#include <iostream>
#include <unistd.h>
#include <signal.h>

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

#define SENSOR_PRINT_ADC_ID 2 

using namespace omnimobot;
using namespace eeros;
using namespace eeros::control;
using namespace eeros::logger;
using namespace eeros::hal;
using namespace eeros::safety;

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
	w.show(0); // ~0 show all messages

	Logger<LogWriter> log;
	
	// Start
	log.info() << "Application OmniMoBot started...";
	
	
	// Initialize hardware
	log.info() << "Initializing hardware";
	HAL& hal = HAL::instance();
	
	log.trace() << "  Creating device structure...";
	FlinkDevice onBoard (ON_BOAD_FPGA_DEVICE);
	
	// Hallsensoren
	FlinkAnalogIn hall0("hall_0", &onBoard, SENSOR_PRINT_ADC_ID,0);
	FlinkAnalogIn hall1("hall_1", &onBoard, SENSOR_PRINT_ADC_ID,1);
	FlinkAnalogIn hall2("hall_2", &onBoard, SENSOR_PRINT_ADC_ID,2);
	FlinkAnalogIn hall3("hall_3", &onBoard, SENSOR_PRINT_ADC_ID,3);

	// Homingsensoren
	FlinkDigIn homeSensor1("Rad1", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD1);
	FlinkDigIn homeSensor2("Rad2", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD2);
	FlinkDigIn homeSensor3("Rad3", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD3);
	
	// HMI
	FlinkDigIn taster1("approval", &onBoard, ON_BOARD_IN_IO_ID, 2);
	FlinkDigIn taster2("driveStop", &onBoard, ON_BOARD_IN_IO_ID, 0); 
	FlinkDigIn taster3("emergencyReset", &onBoard, ON_BOARD_IN_IO_ID, 6); 
	FlinkDigIn taster4("approvalPendulum", &onBoard, ON_BOARD_IN_IO_ID, 4);
	
	// Notstop
	FlinkDigIn emergencyStop("emergencyStop", &onBoard, ON_BOARD_IN_IO_ID, 11); 
	
	FlinkDigOut emergencyLED("emergencyLED", &onBoard, ON_BOARD_OUT_IO_ID, 0);
	FlinkDigOut stopDriveLED("stopDriveLED", &onBoard, ON_BOARD_OUT_IO_ID, 2);
	FlinkDigOut approvalLED("approvalLED", &onBoard, ON_BOARD_OUT_IO_ID, 1);// Taster 1
	FlinkDigOut emergencyResetLED("emergencyResetLED", &onBoard, ON_BOARD_OUT_IO_ID, 4); // Taster 3
	FlinkDigOut powerOnLED("powerOnLED", &onBoard, ON_BOARD_OUT_IO_ID, 3);

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

 	// Watchdog
	FlinkWatchdog watchdog("watchdog", &onBoard, ON_BOARD_WATCHDOG_ID,0.1); // [s]// braucht min. alle 0.009 s

	
	// Im Hal dazu fuegen
	hal.addPeripheralInput(&hall0); 
	hal.addPeripheralInput(&hall1);
	hal.addPeripheralInput(&hall2);
	hal.addPeripheralInput(&hall3);
	
	hal.addPeripheralInput(&homeSensor1);
	hal.addPeripheralInput(&homeSensor2);
	hal.addPeripheralInput(&homeSensor3);
	
	hal.addPeripheralInput(&taster1);
	hal.addPeripheralInput(&taster2);
	hal.addPeripheralInput(&taster3);
	hal.addPeripheralInput(&taster4);
	hal.addPeripheralInput(&emergencyStop);
	
	hal.addPeripheralOutput(&stopDriveLED);
	hal.addPeripheralOutput(&emergencyLED);
	hal.addPeripheralOutput(&approvalLED);
	hal.addPeripheralOutput(&powerOnLED);
	hal.addPeripheralOutput(&emergencyResetLED);
	
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
	


	// craete control system
	log.info() << "create control System";
	ControlSystemWithJacobi controlSys(dt); 
	
	
	// create safty system
	log.info() << "create safty System";
	SafetyWithControlJacobiProb properties(&controlSys, watchdog);
	SafetySystem safetySystem(properties, dt); 
	
	while(running && !controlSys.joystick.shoutDown()) {
		double slip = controlSys.jacobi.getSlip();
		if(slip > 100.0) {
			std::cout << "Slips: "<< slip << std::endl;
		}
		sleep(1);
	}

	safetySystem.triggerEvent(event::doControlStop);
	
	sleep(1);
		
	safetySystem.shutdown();

		
		// pwm reset
	pwmAaxis0.setDutyCycle(0.0);
	log.info() << pwmAaxis0.get();
	pwmAaxis1.setDutyCycle(0.0);
	pwmAaxis2.setDutyCycle(0.0);
	pwmAaxis3.setDutyCycle(0.0);
	pwmAaxis4.setDutyCycle(0.0);
	pwmAaxis5.setDutyCycle(0.0);
	pwmBaxis0.setDutyCycle(0.0);
	pwmBaxis1.setDutyCycle(0.0);
	pwmBaxis2.setDutyCycle(0.0);
	pwmBaxis3.setDutyCycle(0.0);
	pwmBaxis4.setDutyCycle(0.0);
	pwmBaxis5.setDutyCycle(0.0);

	emergencyLED.set(false);
	stopDriveLED.set(false);
	approvalLED.set(false);
	emergencyResetLED.set(false);
	powerOnLED.set(false);
	
	log.trace() << "exiting application";
    return 0;
}
