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
#include <eeros/math/Matrix.hpp>

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

	Worker(FlinkAnalogIn& hall_0, FlinkAnalogIn& hall_1, FlinkAnalogIn& hall_2, FlinkAnalogIn& hall_3, FlinkAnalogIn& acc_y, FlinkAnalogIn& acc_x, FlinkAnalogIn& acc_z,FlinkAnalogIn& GND, double dt) : 
	hall0(hall_0), 
	hall1(hall_1),  
	hall2(hall_2), 
	hall3(hall_3),
	accy(acc_y),
	accx(acc_x),
	accz(acc_z), 
	gnd(GND), 
	counter(50000),

	PeriodicThread(dt, 0, true, paused) {

		hallIn.zero();
		hallValuePrev.zero();
// 		fileout.open("/home/stefan/Projects/omnimobot/test/data/HallOut.txt");
		fileout.open("HallOut.txt");
	}
	
	void run() {

		hallIn(0) = hall0.get() * 4095.0;
		hallIn(1) = hall1.get() * 4095.0;
		hallIn(2) = hall2.get() * 4095.0;
		hallIn(3) = hall3.get() * 4095.0;
		hallIn(4) = accx.get() * 4095.0;
		hallIn(5) = accy.get() * 4095.0;
		hallIn(6) = accz.get() * 4095.0;
		hallIn(7) = gnd.get() * 4095.0;
		
		fileout << hallIn(0) << "\t" << hallIn(1) << "\t" << hallIn(2)<< "\t" << hallIn(3)<< "\t" << hallIn(4)<< "\t" << hallIn(5)<< "\t" << hallIn(6)<< "\t" << hallIn(7);

		
				// Filter
		for (int i = 0; i < 4; i++) {
		
			if (i == 0 && hallIn(i) < 2015) {
				hallIn(i) = hallValuePrev(i);
			}
			if (i == 1 && hallIn(i) < 2000) {
				hallIn(i) = hallValuePrev(i);
			}
			
			if (i == 2 && hallIn(i) < 2032) {
				hallIn(i) = hallValuePrev(i);
			}
			
			if (i == 3 && hallIn(i) < 2015) {
				hallIn(i) = hallValuePrev(i);
			}
			
			// so nicht
// 			if (hallIn(i) < hallValuePrev(i) - filtervalue || hallIn(i) > hallValuePrev(i) + filtervalue) {
// 				hallIn(i) = hallValuePrev(i);
// 			}
			

			
			if (i == 0 && hallIn(i) > 2390) {
				hallIn(i) = hallValuePrev(i);

			}
			
			if (i == 1 && hallIn(i) > 2400) {
				hallIn(i) = hallValuePrev(i);

			}
			
			if (i == 2 && hallIn(i) > 2390) {
				hallIn(i) = hallValuePrev(i);

			}

			if (i == 3 && hallIn(i) > 2380) {
				hallIn(i) = hallValuePrev(i);

			}
		}
		
// 		// Filter alt
// 		for (int i = 0; i < 4; i++) {
// 		
// 			if (hallIn(i) < 2050) {
// 				hallIn(i) = hallValuePrev(i);
// 
// 			}
// 			
// 			if (i == 0 && hallIn(i) > 2390) {
// 				hallIn(i) = hallValuePrev(i);
// 
// 			}
// 			
// 			if (i == 1 && hallIn(i) > 2400) {
// 				hallIn(i) = hallValuePrev(i);
// 
// 			}
// 			
// 			if (i == 2 && hallIn(i) > 2390) {
// 				hallIn(i) = hallValuePrev(i);
// 
// 			}
// 
// 			if (i == 3 && hallIn(i) > 2380) {
// 				hallIn(i) = hallValuePrev(i);
// 
// 			}
// 		}
		
		fileout  << "\t" << hallIn(0) << "\t" << hallIn(1) << "\t" << hallIn(2)<< "\t" << hallIn(3)<< "\t" << hallIn(4)<< "\t" << hallIn(5)<< "\t" << hallIn(6)<< "\t" << hallIn(7)<< std::endl;
		
		counter.count();
		
		if (counter.isCountEnd()) {
			
			fileout.close();
			stop();
		}

		
		
		
		
		
		
		for (int i = 0; i < 4; i++) {
			hallValuePrev(i) = hallIn(i);
		}
		
	}
	
protected:
	std::ofstream fileout;
	
	
private:
	
	FlinkAnalogIn& hall0;
	FlinkAnalogIn& hall1;
	FlinkAnalogIn& hall2;
	FlinkAnalogIn& hall3;
	FlinkAnalogIn& accy;
	FlinkAnalogIn& accx;
	FlinkAnalogIn& accz;
	FlinkAnalogIn& gnd;
	
	eeros::math::Vector<8> hallIn;
	eeros::math::Vector<4> hallValuePrev;
	
	Counter counter;
	
	static constexpr int filtervalue = 50;


};





int main() {
	std::cout << "HallSensor Testing " << std::endl<< std::endl;;
	
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
	FlinkAnalogIn gnd("gnd", &onBoardInIOs, SENSOR_PRINT_ADC_ID,7);
	

	// Registrieren 
	std::cout << "  Registering in the HAL..." << std::endl;
	hal.addPeripheralInput(&hall0); // registrieren beim Hal
	hal.addPeripheralInput(&hall1);
	hal.addPeripheralInput(&hall2);
	hal.addPeripheralInput(&hall3);
	hal.addPeripheralInput(&accy);
	hal.addPeripheralInput(&accx);
	hal.addPeripheralInput(&accz);
	hal.addPeripheralInput(&gnd);
	

// 	HAL& hal = HAL::instance();
// 	initHardware();

	log.info() << "hardware configuration done";
	
//	Worker worker(0.001);
	Worker worker(hall0, hall1, hall2, hall3, accy, accx, accz, gnd, 0.001);
	log.info() << "starting worker thread";
	worker.start();
	log.info() << "waiting for worker thread";
	worker.join(); // Wartet solange bis der Thred fertig
	
	std::cout << "Test finished..." << std::endl;
}