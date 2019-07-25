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

#include <eeros/control/DeMux.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/Saturation.hpp>
#include <omnimobot/control/block/I.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Sum.hpp>
#include <omnimobot/constants.hpp>
#include <omnimobot/control/block/MotorModel.hpp>
#include <omnimobot/control/block/VoltageToPWM.hpp>
#include <omnimobot/control/block/InvJacobian.hpp>

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

static eeros::math::Matrix<6,6,double> gear = eeros::math::Matrix<6,6,double>({	18.0, 0.0,  0.0, 0.0,    0.0,    0.0, // (18) gear Antrieb
																						0.0, 18.0,  0.0, 0.0,    0.0,    0.0,
																						0.0,  0.0, 18.0, 0.0,    0.0,    0.0,
																						0.0,  0.0,  0.0, 36.75,  0.0,    0.0, // (36.75) gear Lenkung
																						0.0,  0.0,  0.0, 0.0,    36.75,  0.0,
																						0.0,  0.0,  0.0, 0.0,    0.0,    36.75}).transpose(); // transponiert, da die Matrix 1.Spalte dann 2. Spalte fühlt// Getriebe Übersetungen
																						
		static eeros::math::Matrix<6,6,double> invGear =eeros::math::Matrix<6,6,double>({	1.0/18.0, 0.0,      0.0,      0.0,       0.0,      0.0,
																							0.0,     1.0/18.0, 0.0,      0.0,       0.0,      0.0,
																							0.0,     0.0,      1.0/18.0, 0.0,       0.0,      0.0,
																							0.0,     0.0,      0.0,      1.0/36.75, 0.0,      0.0,
																							0.0,     0.0,      0.0,      0.0,       1.0/36.75,0.0,
																							0.0,     0.0,      0.0,      0.0,       0.0,      1.0/36.75}).transpose();
																							
		static eeros::math::Matrix<6,6,double> inertia = eeros::math::Matrix<6,6,double>({	0.011962,   0.0,       0.0,       0.0,       0.0,       0.0, 
																							0.0,       0.011962,   0.0,       0.0,       0.0,       0.0,
																							0.0,       0.0,       0.011962,   0.0,       0.0,       0.0,
																							0.0,       0.0,       0.0,       0.029557,   0.0,       0.0, 
																							0.0,       0.0,       0.0,       0.0,       0.029557,   0.0,
																							0.0,       0.0,       0.0,       0.0,       0.0,       0.029557}).transpose();				// Massenträgheiten
		
		static eeros::math::Matrix<6,1,double> torqueLim = eeros::math::Matrix<6,1,double>({0.53,
																							0.53,
																							0.53,
																							0.53,
																							0.53,
																							0.53	});								// Momenten Limit [Nm]
		
		static eeros::math::Matrix<6,1,double> resetValue = eeros::math::Matrix<6,1,double>({ 0.0,
																							  0.0,
																								0.0,
																								0.0,
																								0.0,
																								0.0  });


class Controltest : public PeriodicThread {
	public:
		Controltest(double period, FlinkWatchdog& wd) : 


		enc0("q0"),
		enc1("q1"),
		enc2("q2"),
		enc3("q3"),
		enc4("q4"),
		enc5("q5"),
		pwmA0("pwmAaxis0"),
		pwmA1("pwmAaxis1"),
		pwmA2("pwmAaxis2"),
		pwmA3("pwmAaxis3"),
		pwmA4("pwmAaxis4"),
		pwmA5("pwmAaxis5"),
		pwmB0("pwmBaxis0"),
		pwmB1("pwmBaxis1"),
		pwmB2("pwmBaxis2"),
		pwmB3("pwmBaxis3"),
		pwmB4("pwmBaxis4"),
		pwmB5("pwmBaxis5"),
		desired(0.0),
		value(0.0),
		zero(0.0),
		kpGain(kp),
		kdGain(kd),
		gainDeltaQToQpoint(1.0/period),
		ctr3(0),
		wd(wd),
		ctr(45000),
		ctr2(500),
		inertiaRobot(inertia),
		motModel(gear, invGear, torqueLim),
		voltageToPwm(bridgeVoltage),
		notStop(true),
		oldStop(false),
		PeriodicThread(period, 0, true, paused){
			
			HAL& hal = HAL::instance();
			
			pilz = hal.getLogicPeripheralInput("notstop");
			
			muxEncIn.getIn(0).connect(enc0.getOut());
			muxEncIn.getIn(1).connect(enc1.getOut());
			muxEncIn.getIn(2).connect(enc2.getOut());
			muxEncIn.getIn(3).connect(enc3.getOut());
			muxEncIn.getIn(4).connect(enc4.getOut());
			muxEncIn.getIn(5).connect(enc5.getOut());
			
			gainDeltaQToQpoint.getIn().connect(muxEncIn.getOut());
			integratorQpoint.getIn().connect(gainDeltaQToQpoint.getOut());
			
// Test für ohne Jacobi
// 			muxdesiredValue.getIn(0).connect(value.getOut());
// 			muxdesiredValue.getIn(1).connect(value.getOut());
// 			muxdesiredValue.getIn(2).connect(value.getOut());
// 			muxdesiredValue.getIn(3).connect(value.getOut());
// 			muxdesiredValue.getIn(4).connect(value.getOut());
// 			muxdesiredValue.getIn(5).connect(value.getOut());
			
			muxdesiredValue.getIn(0).connect(zero.getOut());
			muxdesiredValue.getIn(1).connect(zero.getOut());
			muxdesiredValue.getIn(2).connect(value.getOut());
			
			posInDeMux.getIn().connect(integratorQpoint.getOut());
	
			posSteerInMux.getIn(0).connect(posInDeMux.getOut(3));
			posSteerInMux.getIn(1).connect(posInDeMux.getOut(4));
			posSteerInMux.getIn(2).connect(posInDeMux.getOut(5));
			
			invjacobi.getInPhiSteer().connect(posSteerInMux.getOut());
			invjacobi.getInVglobal().connect(muxdesiredValue.getOut());
			
			integrator.getIn().connect(invjacobi.getOutOmega());
			
// 			integrator.getIn().connect(muxdesiredValue.getOut());
			
			sum1.getIn(0).connect(integrator.getOut());
			sum1.negateInput(1);
			sum1.getIn(1).connect(integratorQpoint.getOut());
			
			kpGain.getIn().connect(sum1.getOut());
			
			sum2.getIn(0).connect(invjacobi.getOutOmega());
// 			sum2.getIn(0).connect(muxdesiredValue.getOut());
			sum2.getIn(1).connect(kpGain.getOut());
			
			sum3.getIn(0).connect(sum2.getOut());
			sum3.negateInput(1);
			sum3.getIn(1).connect(gainDeltaQToQpoint.getOut());
			
			kdGain.getIn().connect(sum3.getOut());
			
			inertiaRobot.getIn().connect(kdGain.getOut());
			
			motModel.getInTorque().connect(inertiaRobot.getOut());
			motModel.getInSpeed().connect(gainDeltaQToQpoint.getOut());
			
			voltageToPwm.getInVoltage().connect(motModel.getOutVoltage());
			deMuxPwmAOut.getIn().connect(voltageToPwm.getOutPwmA());
			deMuxPwmBOut.getIn().connect(voltageToPwm.getOutPwmB());
			
			pwmA0.getIn().connect(deMuxPwmAOut.getOut(0));		//deMuxPwmAOut.getOut(0)
			pwmA1.getIn().connect(deMuxPwmAOut.getOut(1));
			pwmA2.getIn().connect(deMuxPwmAOut.getOut(2));
			pwmA3.getIn().connect(deMuxPwmAOut.getOut(3));
			pwmA4.getIn().connect(deMuxPwmAOut.getOut(4));
			pwmA5.getIn().connect(deMuxPwmAOut.getOut(5));
                  
			pwmB0.getIn().connect(deMuxPwmBOut.getOut(0));
			pwmB1.getIn().connect(deMuxPwmBOut.getOut(1));
			pwmB2.getIn().connect(deMuxPwmBOut.getOut(2));       
			pwmB3.getIn().connect(deMuxPwmBOut.getOut(3));
			pwmB4.getIn().connect(deMuxPwmBOut.getOut(4));
			pwmB5.getIn().connect(deMuxPwmBOut.getOut(5));
			
			integratorQpoint.setInitCondition(resetValue);
			integrator.setInitCondition(resetValue);
			
			integratorQpoint.enable();
			integrator.enable();
		}
			
		void run() {
			
			
			
			wd.set(true);
			
			ctr3++;
			
			ctr.count();
			
			if(ctr.isCountEnd()){
				
				log.info() << "counter true nach: "<< ctr3;
				stop();
			}
			
			desired = value.getOut().getSignal().getValue(); 
			// Rampe
			if(desired < 1.0){
				value.setValue(desired+0.00005);
			}

			
			value.run();
			zero.run();
			
			enc0.run();
			enc1.run();
			enc2.run();
			enc3.run();
			enc4.run();
			enc5.run();
			muxEncIn.run();
			gainDeltaQToQpoint.run();
			integratorQpoint.run();
			muxdesiredValue.run();
			posInDeMux.run();
			posSteerInMux.run();
			invjacobi.run();
			integrator.run();
			sum1.run();
			kpGain.run();
			sum2.run();
			sum3.run();	
			kdGain.run();
			inertiaRobot.run();
			motModel.run();
			voltageToPwm.run();
			deMuxPwmAOut.run();
			deMuxPwmBOut.run();
			
			pwmA0.run();
			pwmA1.run();
			pwmA2.run();
			pwmA3.run();
			pwmA4.run();
			pwmA5.run();
			     
			pwmB0.run();
			pwmB1.run();
			pwmB2.run();
			pwmB3.run();
		    pwmB4.run();
 			pwmB5.run();

			
			
// 						// Output
// 			ctr2.count();
// 			
// 			if(ctr2.isCountEnd()){
// 				log.info()<< "getOutOmegaRad0: "<< invjacobi.getOutOmegaRad().getSignal().getValue()(0) << "getOutOmegaRad3: "<< invjacobi.getOutOmegaRad().getSignal().getValue()(3) << "  saturaton Out: "<< motModel.torqueSaturation.getOut().getSignal().getValue()(0) << "   sum1 out: "<< sum1.getOut().getSignal().getValue()(0)<< "   sum2 out: "<< sum2.getOut().getSignal().getValue()(0)<< "   sum3 out: "<< sum3.getOut().getSignal().getValue()(0);
// 				ctr2.reset();
// 			}
// 			
// 			notStop = pilz->get(); // immer neu!!!!!
// 			if (notStop != oldStop) {
// 				std::cout << "########################### stop:  " << notStop << "  #######################" << std::endl;
// 			}
// 			oldStop = notStop;
// 		
			
		}
		
		
	private:
			

		
		eeros::control::PeripheralInput<double> enc0;
		eeros::control::PeripheralInput<double> enc1;
		eeros::control::PeripheralInput<double> enc2;
		eeros::control::PeripheralInput<double> enc3;
		eeros::control::PeripheralInput<double> enc4;
		eeros::control::PeripheralInput<double> enc5;
		eeros::control::PeripheralOutput<double> pwmA0;
		eeros::control::PeripheralOutput<double> pwmA1;
		eeros::control::PeripheralOutput<double> pwmA2;
		eeros::control::PeripheralOutput<double> pwmA3;
		eeros::control::PeripheralOutput<double> pwmA4;
		eeros::control::PeripheralOutput<double> pwmA5;
		eeros::control::PeripheralOutput<double> pwmB0;
		eeros::control::PeripheralOutput<double> pwmB1;
		eeros::control::PeripheralOutput<double> pwmB2;
		eeros::control::PeripheralOutput<double> pwmB3;
		eeros::control::PeripheralOutput<double> pwmB4;
		eeros::control::PeripheralOutput<double> pwmB5;
		
		eeros::hal::PeripheralInput<bool>* pilz;
		
		eeros::control::Constant<double> value;
		eeros::control::Constant<double> zero;
		
		eeros::control::Mux<6, double> muxEncIn;
		eeros::control::Mux<3, double> muxdesiredValue;
		eeros::control::Mux<3, double> posSteerInMux;
		eeros::control::DeMux<6, double> deMuxDesiredVeloLocal;
		eeros::control::DeMux<6, double> deMuxPwmAOut;
		eeros::control::DeMux<6, double> deMuxPwmBOut;
		eeros::control::DeMux<6, double> posInDeMux;
		eeros::control::Sum<2, Vector6> sum1;
		eeros::control::Sum<2, Vector6> sum2;
		eeros::control::Sum<2, Vector6> sum3;
		eeros::control::Gain<Vector6, double> gainDeltaQToQpoint;
		omnimobot::I<Vector6> integratorQpoint;
		omnimobot::I<Vector6> integrator;
		eeros::control::Gain<Vector6, double> kpGain;
		eeros::control::Gain<Vector6, double> kdGain;
		eeros::control::Gain<Vector6, eeros::math::Matrix<6,6>> inertiaRobot;
		
		InvJacobian invjacobi;
		
		MotorModel motModel;			//  ?? wie parameter übergeben (wenn leerer konstruktor wird nicht 2 mal geöffnet?)
		VoltageToPWM voltageToPwm;
		
		FlinkWatchdog& wd;
		
		Counter ctr;
		Counter ctr2;
		
		double desired;
		bool oldStop;
		bool notStop;
		
		double angle;	
		int ctr3;	
		int i;	

};

static volatile bool running = true;
void sig_handler(int signum)
{
    running = false;
}

int main(int argc, char *argv[]) 
{
	
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	w.show(~0); // show all messages
	//w.show(3); // show first four levels
	Logger<LogWriter> log;
	
	// Start
	log.info() << "Test drive started...";
	
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
	
	//PWM A
	std::cout << "Setup PWM " << std::endl;
	


	
	double periodTime = 0.001; // [s] weniger wie 0.008 s
	
	log.info() << "creating test thread";
	Controltest test(periodTime,watchdog);
	
	sleep(1);
	
	log.info() << "starting test thread";
	watchdog.reset();
	test.start();
	
	log.info() << "waiting for test thread";
	
	while(running) {
		sleep(1);
		//auto &counter = safetySystem.counter;
		auto &counter = test.counter;
		log.fatal() << "mean: " << counter.run.mean << " max: " << counter.run.max;
	}
	
	
// 	test.join(); // Wartet solange bis der Thred fertig
	
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

	log.trace() << "Test drive ended";
    return 0;
}