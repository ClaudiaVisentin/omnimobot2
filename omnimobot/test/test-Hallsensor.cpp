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
//	Worker(double dt) : sensor("home"), hall0("hall_0"), hall1("hall_1"),  hall2("hall_2"), hall3("hall_3"), phiCx(0.0), phiCy(0.0),  i(0), PeriodicThread(dt, 0, true, paused) {
	
	Worker(FlinkAnalogIn& hall_0, FlinkAnalogIn& hall_1, FlinkAnalogIn& hall_2, FlinkAnalogIn& hall_3, FlinkAnalogIn& acc_y, FlinkAnalogIn& acc_x, FlinkAnalogIn& acc_z, FlinkDigIn& home, double dt) : 
	hall0(hall_0), 
	hall1(hall_1),  
	hall2(hall_2), 
	hall3(hall_3),
	accy(acc_y),
	accx(acc_x),
	accz(acc_z), 
	home(home),
	sensor("home"),
	phiCx(0.0), 
	phiCy(0.0), 
	ctr1(0),
	ctr2(0),
	ctr3(0),
	i(0), 
	counter(25000),
	s0(0.0),
	s1(0.0),
	s2(0.0),
	s3(0.0),
	lostStick(0),
	PeriodicThread(dt, 0, true, paused) {
		hallIn.zero();
		hallValuePrev.fill(2015.0);
		maxWithOutStick.zero();
		hallMin.fill(4095.0);
		hallMax.fill(2000.0);
		mean.zero();
	//	nothing to do
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
		hallIn(0) = hall0.get() * 4095.0;
		hallIn(1) = hall1.get() * 4095.0;
		hallIn(2) = hall2.get() * 4095.0;
		hallIn(3) = hall3.get() * 4095.0;
		
		lostStick = 0;
		
	//	usleep(1200);
		
		// Filter
		for (int i = 0; i < 4; i++) {
		
// 			if (hallIn(i) < 2050) {
// 				hallIn(i) = hallValuePrev(i);
// 				lostStick++;
// 			}
// 			
// 			if (i == 0 && hallIn(i) > 2390) {
// 				hallIn(i) = hallValuePrev(i);
// 				lostStick++;
// 			}
// 			
// 			if (i == 1 && hallIn(i) > 2400) {
// 				hallIn(i) = hallValuePrev(i);
// 				lostStick++;
// 			}
// 			
// 			if (i == 2 && hallIn(i) > 2390) {
// 				hallIn(i) = hallValuePrev(i);
// 				lostStick++;
// 			}
// 
// 			if (i == 3 && hallIn(i) > 2380) {
// 				hallIn(i) = hallValuePrev(i);
// 				lostStick++;
// 			}
			
			//control
// 			if (lostStick == 3){
// 				stop();
// 			}
			
			// min
			if (hallMin(i) > hallIn(i)) {
				hallMin(i) = hallIn(i);
			}
			
			// max
			if (hallMax(i) < hallIn(i)) {
				hallMax(i) = hallIn(i);
			}
						
		}
		
		
	
		for (int i = 0; i < 4; i++) {
			hallValuePrev(i) = hallIn(i);
		}
		
		
		s0 = hallIn(0)- 2020;//mean     max:2033;//2194.0;
		s1 = hallIn(1)- 2013;//2021;//2179.0;
		s2 = hallIn(2)- 2044;//2053;//2210.0;
		s3 = hallIn(3)- 2028;//2037;//2192.0;
		
		phiCx = 0.00175 * (s0 - s2); // 0.00175 ist ein Faktor der von David ermittelt wurde
		phiCy = 0.00175 * (s1 - s3);
		
// 		if(s0 < 2000.0 || s1 < 2000.0 || s2 < 2000.0 || s3 < 2000.0) {
// 			ctr1++;
// 		}
		
		if(hallIn(0) + hallIn(1) +hallIn(2)+ hallIn(3) < 8500.0) {
			maxWithOutStick(0) = s0;
			maxWithOutStick(1) = s1;
			maxWithOutStick(2) = s2;
			maxWithOutStick(3) = s3;
			ctr2++;
		}
		
		for (int i = 0; i < 4; i++) {
			mean(i) = mean(i) + hallIn(i);
		}
		
		
		ctr3++;
		
		if(i > 300){
			std::cout << "Hall0Max: " << hallMax(0)<< "     Hall1Max: " << hallMax(1)<< "    Hall2Max: " << hallMax(2)<< "    Hall3Max: " << hallMax(3);
			std::cout << "                     Hall0Min: " << hallMin(0)<< "     Hall1Min: " << hallMin(1)<< "      Hall2Min: " << hallMin(2)<< "     Hall3Min: " << hallMin(3)<< std::endl;// << std::endl;
// 			std::cout << "Hall0: " << s0<< "      Hall1: " <<s1 << "      Hall2: " << s2 << "      Hall3: " << s3<< " ";// << std::endl;
// 			std::cout << "groesser als 8400: " << ctr2<< "        Hall0: " << s0<< "      Hall1: " <<s1 << "      Hall2: " << s2 << "      Hall3: " << s3<< std::endl;
// 			std::cout << "Hall0: " << maxWithOutStick(0)<< "      Hall1: " <<maxWithOutStick(1) << "      Hall2: " << maxWithOutStick(2) << "      Hall3: " << maxWithOutStick(3) << " ";// << std::endl;
// 			std::cout<<"       phiCx: "<< phiCx<<"       phiCy: "<< phiCy<< std::endl;// << std::endl;
// 			std::cout<<"      mean0: "<<mean(0)/ctr3<<"      mean1: "<<mean(1)/ctr3<<"      mean2: "<<mean(2)/ctr3<<"      mean3: "<<mean(3)/ctr3<<"       phiCx: "<< phiCx<<"       phiCy: "<< phiCy<< std::endl << std::endl;
			
			i = 0;
			ctr1 = 0;
		}
		i++;

		counter.count();
		
		if (counter.isCountEnd()) {
			
			stop();
		}
		
/*		
		hall0.run();
		hall1.run();
		hall2.run();
		hall3.run();
		sensor.run();
		
		phiCx = 0.00175 * (hall0.getOut().getSignal().getValue() - hall2.getOut().getSignal().getValue());
		phiCy = 0.00175 * (hall3.getOut().getSignal().getValue() - hall1.getOut().getSignal().getValue());
		
		if(i > 100){
			std::cout << "Hall0: " << hall0.getOut().getSignal().getValue() << " Hall1: " << hall1.getOut().getSignal().getValue() << " Hall2: " << hall2.getOut().getSignal().getValue() << " Hall3: " << hall3.getOut().getSignal().getValue() << std::endl;
			std::cout << "phiCx in rad: " << phiCx << " phiCy in rad: " << phiCy << std::endl << std::endl;
			i = 0;
		}
		i++;

		if(sensor.getOut().getSignal().getValue()) { // Endschalter erreicht
			stop();
		}*/
		
	}
	
	double getphiCx() {
		return phiCx;
	}
	
	double getphiCy() {
		return phiCx;
	}
	
	
private:
	
	FlinkAnalogIn& hall0;
	FlinkAnalogIn& hall1;
	FlinkAnalogIn& hall2;
	FlinkAnalogIn& hall3;
	FlinkAnalogIn& accy;
	FlinkAnalogIn& accx;
	FlinkAnalogIn& accz;
	
	FlinkDigIn& home;
	
 	eeros::control::PeripheralInput<bool> sensor;
	
	Counter counter;
// 	eeros::control::PeripheralInput<double> hall0;
// 	eeros::control::PeripheralInput<double> hall1;
// 	eeros::control::PeripheralInput<double> hall2;
// 	eeros::control::PeripheralInput<double> hall3;
	
	eeros::math::Vector<4> hallIn;
	eeros::math::Vector<4> hallValuePrev;
	eeros::math::Vector<4> maxWithOutStick;
	eeros::math::Vector<4> hallMin;
	eeros::math::Vector<4> hallMax;
	eeros::math::Vector<4> mean;
	
	int lostStick;
	
	double phiCx;
	double phiCy;
	double s0;
	double s1;
	double s2;
	double s3;
// 	std::atomic<double> phiCx; // atomic verhindert das gleichzeitige zugreifen von zwei thred
//  std::atomic<double> phiCy;
	int ctr1;
	int ctr2;
	int ctr3;
	int i;
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

// 	HAL& hal = HAL::instance();
// 	initHardware();

	log.info() << "hardware configuration done";
	
//	Worker worker(0.001);
	Worker worker(hall0, hall1, hall2, hall3, accy, accx, accz, home, 0.002);
	log.info() << "starting worker thread";
	worker.start();
	log.info() << "waiting for worker thread";
	worker.join(); // Wartet solange bis der Thred fertig
	
	std::cout << "Test finished..." << std::endl;
}