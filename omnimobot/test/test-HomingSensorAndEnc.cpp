#include <iostream>
#include <ostream>
#include <unistd.h>
#include <atomic>

#include <eeros/hal/HAL.hpp>
#include <eeros/hal/FlinkFqd.hpp>
#include <eeros/hal/FlinkDigIn.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/PeriodicThread.hpp>
#include <omnimobot/Counter.hpp>
#include <omnimobot/control/block/I.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Mux.hpp>

#include <eeros/control/PeripheralInput.hpp>

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


class Worker : public PeriodicThread {
public:
	Worker(double dt) : 
	enc0("q0"),
	enc1("q1"),
	enc2("q2"),
	enc3("q3"),
	enc4("q4"),
	enc5("q5"),
	angle0(0.0),
	angle1(0.0),
	angle2(0.0),
	angle3(0.0),
	angle4(0.0),
	angle5(0.0),
	ctr(40000),
	gainDeltaQToQpoint(1.0/dt),
	i(0), 
	PeriodicThread(dt, 0, true, paused) {
		HAL& hal = HAL::instance();
		
		muxEncIn.getIn(0).connect(enc0.getOut());
		muxEncIn.getIn(1).connect(enc1.getOut());
		muxEncIn.getIn(2).connect(enc2.getOut());
		muxEncIn.getIn(3).connect(enc3.getOut());
		muxEncIn.getIn(4).connect(enc4.getOut());
		muxEncIn.getIn(5).connect(enc5.getOut());
		
		gainDeltaQToQpoint.getIn().connect(muxEncIn.getOut());
		integrator.getIn().connect(gainDeltaQToQpoint.getOut());
		
		integrator.enable();
	}
	
	void run() {
		
		enc0.run();
		enc1.run();
		enc2.run();
		enc3.run();
		enc4.run();
		enc5.run();
		muxEncIn.run();
		gainDeltaQToQpoint.run();
		integrator.run();
		
		angle0 = integrator.getOut().getSignal().getValue()(0);
		angle1 = integrator.getOut().getSignal().getValue()(1);
		angle2 = integrator.getOut().getSignal().getValue()(2);
		angle3 = integrator.getOut().getSignal().getValue()(3);
		angle4 = integrator.getOut().getSignal().getValue()(4);
		angle5 = integrator.getOut().getSignal().getValue()(5);
		
		ctr.count();
		
		if(ctr.isCountEnd()) { // Endschalter erreicht
			stop();
		}
		
		if(i > 100){
// 			std::cout << angle0 << "    ";
// 			std::cout << angle1 << "    "; 
// 			std::cout << angle2 << "    ";
			std::cout << angle3 << "    ";
			std::cout << angle4 << "    ";
			std::cout << angle5 << std::endl;
			i = 0;
		}
		i++;
	}
	
	double getAngle() {
		return angle0;
	}
	
private:
	eeros::control::PeripheralInput<double> enc0;
	eeros::control::PeripheralInput<double> enc1;
	eeros::control::PeripheralInput<double> enc2;
	eeros::control::PeripheralInput<double> enc3;
	eeros::control::PeripheralInput<double> enc4;
	eeros::control::PeripheralInput<double> enc5;
	omnimobot::I<eeros::math::Vector<6>> integrator;
	eeros::control::Mux<6, double> muxEncIn;
	eeros::control::Gain<eeros::math::Vector<6>, double> gainDeltaQToQpoint;
//	std::atomic<double> angle; // atomic verhindert das gleichzeitige zugreifen von zwei thred
	double angle0; 
	double angle1; 
	double angle2; 
	double angle3; 
	double angle4; 
	double angle5; 
	
	
	Counter ctr;
	int i;
};

static volatile bool running = true;
void sig_handler(int signum)
{
    running = false;
}

int main() {
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	
	log.info() << "Encoder Test Application started...";
	
	log.info() << "configuring hardware";
	HAL& hal = HAL::instance();
	
	FlinkDevice onBoard (ON_BOAD_FPGA_DEVICE);
	
	// Encoder
	FlinkFqd enc_0("q0", &onBoard, ON_BOARD_FQD_ID, 0, 6.28318530718 / (4 * 1024*18), 0,true); // 36.75 ist die Getriebe übersetzung Lenkung, 18:antrieb
	FlinkFqd enc_1("q1", &onBoard, ON_BOARD_FQD_ID, 1, 6.28318530718 / (4 * 1024*18), 0,true); // true: delta q wird ausgegeben
	FlinkFqd enc_2("q2", &onBoard, ON_BOARD_FQD_ID, 2, 6.28318530718 / (4 * 1024*18), 0,true);
	FlinkFqd enc_3("q3", &onBoard, ON_BOARD_FQD_ID, 3, 6.28318530718 / (4 * 1024*36.75), 0,true);
	FlinkFqd enc_4("q4", &onBoard, ON_BOARD_FQD_ID, 4, 6.28318530718 / (4 * 1024*36.75), 0,true);
	FlinkFqd enc_5("q5", &onBoard, ON_BOARD_FQD_ID, 5, 6.28318530718 / (4 * 1024*36.75), 0,true);
	
		// homing sensor
	FlinkDigIn homeSensor1("Wheel1", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD1); // zum Invertieren letzten Parameter auf true setzen, Zahl ist welcher Sensorbenutzt wird
	FlinkDigIn homeSensor2("Wheel2", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD2); // zum Invertieren letzten Parameter auf true setzen, Zahl ist welcher Sensorbenutzt wird
	FlinkDigIn homeSensor3("Wheel3", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD3); // zum Invertieren letzten Parameter auf true setzen, Zahl ist welcher Sensorbenutzt wird
	
	
	
	hal.addPeripheralInput(&enc_0);
	hal.addPeripheralInput(&enc_1);
	hal.addPeripheralInput(&enc_2);
	hal.addPeripheralInput(&enc_3);
	hal.addPeripheralInput(&enc_4);
	hal.addPeripheralInput(&enc_5);
	hal.addPeripheralInput(&homeSensor1);
	hal.addPeripheralInput(&homeSensor2);
	hal.addPeripheralInput(&homeSensor3);
	
	
	log.info() << "hardware configuration done";
	
	log.info() << "creating worker thread";
	Worker worker(0.001);
	
	log.info() << "starting worker thread";
	worker.start();
	log.info() << "waiting for worker thread";
	worker.join(); // Wartet solange bis der Thred fertig
	log.info() << "worker thread finished";
	
	log.info() << "Angle = " << worker.getAngle();
	
	log.info() << "Encoder Test Application ended";
}
