#include "ControlsystemDummy.hpp"
#include "SafetyWithoutControlSystem.hpp"

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
#include <eeros/hal/DummyLogicOutput.hpp>
#include <eeros/core/PeriodicThread.hpp>

#include <iostream>
#include <unistd.h>
#include <signal.h>

#define ON_BOAD_FPGA_DEVICE "/dev/flink0"
#define ON_BOARD_PWM_ID 0
#define SENSOR_PRINT_ADC_ID 1
#define ON_BOARD_FQD_ID 2
#define ON_BOARD_OUT_IO_ID 3
#define ON_BOARD_IN_IO_ID 4

#define MOTOR_BRIDGE_DEVICE "/dev/flink1"
#define MOTOR_BRIDGE_PWM_A_ID 0 // alle in die einte richtung drehen (motoren)
#define MOTOR_BRIDGE_PWM_B_ID 1
#define MOTOR_BRIDGE_WATCHDOG_ID 2
#define MOTOR_BRIDGE_FQD_ID 3
#define MOTOR_BRIDGE_LED_ID 4

#define HOMSENSOR_RAD1 14
#define HOMSENSOR_RAD2 15
#define HOMSENSOR_RAD3 12

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
	w.show(~0); // show all messages
	//w.show(3); // show first four levels
	Logger<LogWriter> log;
	
	// Start
	log.info() << "Application OmniMoBot started...";
	
	// Initialize hardware
	log.info() << "Initializing hardware";
	
	HAL& hal = HAL::instance();
	
	log.trace() << "  Creating device structure...";
	FlinkDevice onBoard (ON_BOAD_FPGA_DEVICE);
	FlinkDevice motorBridge (MOTOR_BRIDGE_DEVICE);
	log.trace() << "  Creating device structure...gut";
	
	// HMI
	FlinkDigIn taster1("approval", &onBoard, ON_BOARD_IN_IO_ID, 0); // Anhängen und Kontrollieren
	FlinkDigIn taster2("emergency", &onBoard, ON_BOARD_IN_IO_ID, 1); // Noch mit Piltz verknüpfen
	FlinkDigIn taster3("emergencyStop", &onBoard, ON_BOARD_IN_IO_ID, 2); // Anhängen und Kontrollieren
	log.trace() << "2";
	
	FlinkDigOut emergency_LED("emrgencyLED", &onBoard, ON_BOARD_OUT_IO_ID, 8);//3
	FlinkDigOut approval_LED("approvalLED", &onBoard, ON_BOARD_OUT_IO_ID, 9);// Taster 1
	FlinkDigOut emergencyStop_LED("emergencyStop_LED", &onBoard, ON_BOARD_OUT_IO_ID, 11); // Taster 3
	FlinkDigOut powerOnLED("powerOnLED", &onBoard, ON_BOARD_OUT_IO_ID, 10);//7
	log.trace() << "3";
	
	// Encoder
	FlinkFqd enc_0("q0", &motorBridge, MOTOR_BRIDGE_FQD_ID, 0, 6.28318530718 / (4 * 1024*18), 0); // 36.75 ist die Getriebe übersetzung Lenkung, 18:antrieb
	FlinkFqd enc_1("q1", &motorBridge, MOTOR_BRIDGE_FQD_ID, 1, 6.28318530718 / (4 * 1024*18), 0);
	FlinkFqd enc_2("q2", &motorBridge, MOTOR_BRIDGE_FQD_ID, 2, 6.28318530718 / (4 * 1024*18),0);
	FlinkFqd enc_3("q3", &motorBridge, MOTOR_BRIDGE_FQD_ID, 3, 6.28318530718 / (4 * 1024*36.75),0);
	FlinkFqd enc_4("q4", &motorBridge, MOTOR_BRIDGE_FQD_ID, 4, 6.28318530718 / (4 * 1024*36.75),0);
	FlinkFqd enc_5("q5", &motorBridge, MOTOR_BRIDGE_FQD_ID, 5, 6.28318530718 / (4 * 1024*36.75),0);
	log.trace() << "5";
	
	// Watchdog
 	FlinkWatchdog motorBoardWatchdog("motorBoardWatchdog", &motorBridge, 2, 0);
 	motorBoardWatchdog.set(2); // Hz
	log.trace() << "8";
	
// 	SysFsDigOut linuxDigOut("watchdog", 248); // watchdog wird inetialisiert (auf pin 248)// referenz im safty
	DummyLogicOutput linuxDigOut("watchdog");
	log.trace() << "9";
	
	
	// Im Hal dazu fuegen
	hal.addPeripheralInput(&taster1);
	hal.addPeripheralInput(&taster2);
	hal.addPeripheralInput(&taster3);
	
	hal.addPeripheralOutput(&emergency_LED);
	hal.addPeripheralOutput(&approval_LED);
	hal.addPeripheralOutput(&powerOnLED);
	hal.addPeripheralOutput(&emergencyStop_LED);
	
	hal.addPeripheralInput(&enc_0);
	hal.addPeripheralInput(&enc_1);
	hal.addPeripheralInput(&enc_2);
	hal.addPeripheralInput(&enc_3);
	hal.addPeripheralInput(&enc_4);
	hal.addPeripheralInput(&enc_5);
	
	// craete control system
	log.info() << "create control System";
	ControlsystemDummy controlSys(0.001); // 0.001 = dt
	
	
	// create safty system
	log.info() << "create safty System";
	SafetyWithoutControlSystem properties(&controlSys, &motorBoardWatchdog);
	SafetySystem safetySystem(properties, 0.001); // 0.001 = dt
	
	//	sleep(1200);
 	while (running) 
		
	while(safetySystem.getCurrentLevel().getId()== 200) sleep(1); // funktion?
	
	safetySystem.triggerEvent(event::doControlStopp);
	
	safetySystem.triggerEvent(event::doOff);
	
	safetySystem.shutdown();
	
	log.trace() << "exiting application";
    return 0;
}
	