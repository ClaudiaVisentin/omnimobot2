#include <omnimobot/control/block/HallSensorInput.hpp>

#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <atomic>
#include <omnimobot/Counter.hpp>

#include <eeros/hal/HAL.hpp>
#include <eeros/hal/FlinkAnalogIn.hpp>
#include <eeros/hal/FlinkDigIn.hpp>
#include <eeros/control/PeripheralInput.hpp>

#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/PeriodicThread.hpp>

#define ON_BOAD_FPGA_DEVICE "/dev/flink0"
// #define ON_BOARD_PWM_ID 0
// #define ON_BOARD_WATCHDOG_ID 1
#define SENSOR_PRINT_ADC_ID 2 // oder 4
// #define ON_BOARD_FQD_ID 5
// #define ON_BOARD_OUT_IO_ID 6
#define ON_BOARD_IN_IO_ID 5


#define HOMSENSOR_RAD1 10
// #define HOMSENSOR_RAD2 14
// #define HOMSENSOR_RAD3 12
using namespace omnimobot;
using namespace eeros;
using namespace eeros::hal;
using namespace eeros::logger;
using namespace eeros::control;

class Worker : public PeriodicThread {
public:
//	Worker(double dt) : sensor("home"), hall0("hall_0"), hall1("hall_1"),  hall2("hall_2"), hall3("hall_3"), phiCx(0.0), phiCy(0.0),  i(0), PeriodicThread(dt, 0, true, paused) {
	
	Worker( double dt) : 
	accy("acc_y"),
	accx("acc_x"),
	accz("acc_z"), 
	sensor("home"),
	phiCx(0.0), 
	phiCy(0.0),  
	i(0), 
	counter(30000),
	s0(0.0),
	s1(0.0),
	s2(0.0),
	s3(0.0),
	PeriodicThread(dt, 0, true, paused) {
// 		hallblock;
// 		hallblock.getPhixyOutput().getSignal().setName("Phi Hall x,y");
	}
	
	void run() {
		// Silvan:
// 		double magFluxMin = 10000000;				// wenn kein Magnet erkannt wird --> minimalem Wert (MUSS NACHGEPRUEFT WERDEN WAS FUER EIN WERT!!!)
		
// 		if(ScaraDriver.getDIButton(2) && hallSens0+hallSens1+hallSens2+hallSens3 >= magFluxMin){	// button 3 (right, blue) balancing and pendulum detected
// 			System.out.print("Go to Center Done");
// 			state = BALANCING_STATE;
// 		}
		
// 		if(hallSens0+hallSens1+hallSens2+hallSens3 <= magFluxMin ){			// if no pendelum (magnet) is detected --> Stop
// 			state = STOP_STATE;
// 		}
// 		hallblock.run();

		
// 		phiCx = hallblock.getPhixyOutput().getSignal().getValue()(0);
// 		phiCy = hallblock.getPhixyOutput().getSignal().getValue()(1);
		
		
		
		if(i > 300){
			std::cout << "      phiCx in rad: " << phiCx << "      phiCy in rad: " << phiCy << std::endl;// << std::endl;
			i = 0;
		}
		i++;

		counter.count();
		
		if (counter.isCountEnd()) {
			stop();
		}
		
		
	}
	
	double getphiCx() {
		return phiCx;
	}
	
	double getphiCy() {
		return phiCx;
	}
	
	
private:

// 	omnimobot::HallSensorInput hallblock;


	
 	eeros::control::PeripheralInput<bool> sensor;
	
	eeros::control::PeripheralInput<double> accy;
	eeros::control::PeripheralInput<double> accx;
	eeros::control::PeripheralInput<double> accz;
	
	Counter counter;
	
	double phiCx;
	double phiCy;
	double s0;
	double s1;
	double s2;
	double s3;

	int i;
};





int main() {
	std::cout << "HallSensorblock Testing " << std::endl<< std::endl;;
	
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	
	log.info() << "Initializing hardware";
	
	HAL& hal = HAL::instance();
	
	FlinkDevice onBoardInIOs(ON_BOAD_FPGA_DEVICE);
	
	FlinkAnalogIn hall0("hall_0", &onBoardInIOs, SENSOR_PRINT_ADC_ID,0);
	FlinkAnalogIn hall1("hall_1", &onBoardInIOs, SENSOR_PRINT_ADC_ID,1);
	FlinkAnalogIn hall2("hall_2", &onBoardInIOs, SENSOR_PRINT_ADC_ID,2);
	FlinkAnalogIn hall3("hall_3", &onBoardInIOs, SENSOR_PRINT_ADC_ID,3);
	FlinkAnalogIn accy("acc_y", &onBoardInIOs, SENSOR_PRINT_ADC_ID,4);
	FlinkAnalogIn accx("acc_x", &onBoardInIOs, SENSOR_PRINT_ADC_ID,5);
	FlinkAnalogIn accz("acc_z", &onBoardInIOs, SENSOR_PRINT_ADC_ID,6);
	
	FlinkDigIn home("home", &onBoardInIOs, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD1); // zum Invertieren letzten Parameter auf true setzen, Zahl ist welcher Sensorbenutzt wird
	
	// Registrieren 
	std::cout << "  Registering in the HAL..." << std::endl;
	hal.addPeripheralInput(&hall0); // registrieren beim Hal
	hal.addPeripheralInput(&hall1);
	hal.addPeripheralInput(&hall2);
	hal.addPeripheralInput(&hall3);
	hal.addPeripheralInput(&accy);
	hal.addPeripheralInput(&accx);
	hal.addPeripheralInput(&accz);
	
	hal.addPeripheralInput(&home);


	log.info() << "hardware configuration done";
	
	
	Worker worker( 0.001);
	log.info() << "starting worker thread";
	worker.start();
	log.info() << "waiting for worker thread";
	worker.join(); // Wartet solange bis der Thred fertig
	
	std::cout << "Test finished..." << std::endl;
}