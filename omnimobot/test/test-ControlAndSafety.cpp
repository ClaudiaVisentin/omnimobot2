#include <omnimobot/control/ControlSystem.hpp>
#include <omnimobot/safety/OmniSafetyProperties.hpp>
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

#include <ostream>
#include <fstream>


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

#define SENSOR_PRINT_ADC_ID 2 // oder 4

using namespace omnimobot;
using namespace eeros;
using namespace eeros::control;
using namespace eeros::logger;
using namespace eeros::hal;
using namespace eeros::math;
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
// 	w.show(~0); // show all messages
 	w.show(0); // show first four levels
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
	FlinkWatchdog watchdog("watchdog", &onBoard, ON_BOARD_WATCHDOG_ID,0.5); // [s]// braucht min. alle 0.009 s vorher 2

	
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
	
	// balance data
	std::ofstream fileBalanceL;
	fileBalanceL.open("BalanceData.txt");
	
	
	double periodTime = dt; // [s] weniger wie 0.008 s

	// craete control system
	log.info() << "create control System";
	ControlSystem controlSys(periodTime); 
	
	
	// create safty system
	log.info() << "create safty System";
	OmniSafetyProperties properties(&controlSys, watchdog);
	SafetySystem safetySystem(properties, periodTime); 

	
	double bufferTipA0[sizeDatalog];
	double bufferTipA1[sizeDatalog];
	
	double bufferTipD0[sizeDatalog];
	double bufferTipD1[sizeDatalog];
	
	double bufferPhiA0[sizeDatalog];
	double bufferPhiA1[sizeDatalog];
	
	double bufferPhiD0[sizeDatalog];
	double bufferPhiD1[sizeDatalog];
	
	double bufferVeloA0[sizeDatalog];
	double bufferVeloA1[sizeDatalog];
	
	double bufferVeloD0[sizeDatalog];
	double bufferVeloD1[sizeDatalog];
	
	double bufferPosA0[sizeDatalog];
	double bufferPosA1[sizeDatalog];
	

	while(running && !controlSys.joystick.shoutDown()) {
		sleep(1);

	}
	
	if (controlSys.pendulumControl.measuringTipActual.getMeasuringData(bufferTipA0,bufferTipA1) && controlSys.measuringPhiHall.getMeasuringData(bufferPhiA0,bufferPhiA1) && controlSys.measuringPhides.getMeasuringData(bufferPhiD0,bufferPhiD1) && controlSys.pendulumControl.measuringTipDes.getMeasuringData(bufferTipD0,bufferTipD1) && controlSys.measuringVeloOut.getMeasuringData(bufferVeloA0,bufferVeloA1) && controlSys.measuringVeloIn.getMeasuringData(bufferVeloD0,bufferVeloD1)&& controlSys.pendulumControl.measuringPosActual.getMeasuringData(bufferPosA0,bufferPosA1)) {

		for (int i = 0; i < sizeDatalog; i++) {
			fileBalanceL << bufferTipA0[i] << '\t' << bufferTipA1[i] << '\t' << bufferTipD0[i]<< '\t' << bufferTipD1[i]<< '\t' << bufferPhiA0[i]<< '\t' << bufferPhiA1[i]<< '\t' << bufferPhiD0[i]<< '\t' << bufferPhiD1[i]<< '\t' << bufferVeloA0[i]<< '\t' << bufferVeloA1[i]<< '\t' << bufferVeloD0[i]<< '\t' << bufferVeloD1[i]<< '\t' << bufferPosA0[i]<< '\t' << bufferPosA1[i]<< std::endl; // 0 is actual tip, 1: desired, 2: actual Phi, 3: deasired Phi
		}
	}
	
	log.info() <<" Slip: "<< controlSys.jacobi.getSlip();
	
	sleep (1);
	
	safetySystem.triggerEvent(event::doControlStop);

	sleep (1);
		
	safetySystem.shutdown();

		
		
		
		// pwm reset
	pwmAaxis0.setDutyCycle(0.0);

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
	
	fileBalanceL.close();
	
	

	log.trace() << "exiting application";
    return 0;
}
