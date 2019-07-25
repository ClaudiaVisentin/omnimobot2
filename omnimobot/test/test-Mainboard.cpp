#include <eeros/hal/HAL.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/FlinkDevice.hpp>
#include <eeros/hal/FlinkFqd.hpp>
#include <eeros/hal/FlinkPwm.hpp>
#include <eeros/hal/FlinkWatchdog.hpp>
#include <eeros/core/PeriodicThread.hpp>
#include <eeros/hal/FlinkDigIn.hpp>
#include <eeros/hal/FlinkAnalogIn.hpp>
#include <eeros/hal/FlinkDigOut.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/Constant.hpp>

#include <iostream>
#include <unistd.h>


#define ON_BOAD_FPGA_DEVICE "/dev/flink0"

#define ON_BOARD_PWM_ID 0
#define ON_BOARD_WATCHDOG_ID 1
#define SENSOR_PRINT_ADC_ID 2	// funktioniert nicht
#define ON_BOARD_AOUT_ID 3
#define ON_BOARD_AIN_ID 4
#define ON_BOARD_FQD_ID 5
#define ON_BOARD_OUT_IO_ID 6
#define ON_BOARD_IN_IO_ID 7

#define HOMSENSOR_RAD1 14
#define HOMSENSOR_RAD2 15
#define HOMSENSOR_RAD3 12


using namespace eeros;
using namespace eeros::logger;
using namespace eeros::hal;
using namespace eeros::control;


class Worker : public PeriodicThread {
	public:
		Worker(double period):
// 			enc0("q0"),
// 			enc1("q1"),
// 			enc2("q2"),
// 			enc3("q3"),
// 			enc4("q4"),
// 			enc5("q5"),
// 			pwm0("pwmAxis0"),
// 			pwm1("pwmAxis1"),
// 			pwm2("pwmAxis2"),
// 			pwm3("pwmAxis3"),
// 			pwm4("pwmAxis4"),
// 			pwm5("pwmAxis5"),
// 			wd("watchd"),
// 			home1("rad1"),
// 			home2("rad2"),
// 			home3("rad3"),
// 			taster1("t1"),
// 			taster2("t2"),
// 			taster3("t3"),
// 			taster4("t4"),
// 			led1("led1"),
// 			led2("led2"),
// 			led3("led3"),
// 			led4("led4"),
// 			hall0("hall_0"),
// 			hall1("hall_1"),
// 			hall2("hall_2"),
// 			hall3("hall_3"),
// 			accy("acc_y"),
// 			accx("acc_x"),
// 			accz("acc_z"),
			PeriodicThread(period, 0, true, paused){   
				
				HAL& hal = HAL::instance();
				
				enc0 = hal.getRealPeripheralInput("q0");
				enc1 = hal.getRealPeripheralInput("q1");
				enc2 = hal.getRealPeripheralInput("q2");
				enc3 = hal.getRealPeripheralInput("q3");
				enc4 = hal.getRealPeripheralInput("q4");
				enc5 = hal.getRealPeripheralInput("q5");
				
				pwm0 = hal.getRealPeripheralOutput("pwmAxis0");
				pwm1 = hal.getRealPeripheralOutput("pwmAxis1");
				pwm2 = hal.getRealPeripheralOutput("pwmAxis2");
				pwm3 = hal.getRealPeripheralOutput("pwmAxis3");
				pwm4 = hal.getRealPeripheralOutput("pwmAxis4");
				pwm5 = hal.getRealPeripheralOutput("pwmAxis5");

 				wd = hal.getLogicPeripheralOutput("watchd");
				
				homeSensorRad1= hal.getLogicPeripheralInput("rad1");
				homeSensorRad2= hal.getLogicPeripheralInput("rad2");
				homeSensorRad3= hal.getLogicPeripheralInput("rad3");
				
				taster1= hal.getLogicPeripheralInput("t1");
				taster2= hal.getLogicPeripheralInput("t2");
				taster3= hal.getLogicPeripheralInput("t3");
				taster4= hal.getLogicPeripheralInput("t4");
				
				led1 = hal.getLogicPeripheralOutput("led1");
				led2 = hal.getLogicPeripheralOutput("led2");
				led3 = hal.getLogicPeripheralOutput("led3");
				led4 = hal.getLogicPeripheralOutput("led4");
		}
		
		void run() {
			
			enc0->get();
			enc1->get();
			enc2->get();
			enc3->get();
			enc4->get();
			enc5->get();
			
			pwm0->set(0.08);
			pwm1->set(0.08);
			pwm2->set(0.08);
			pwm3->set(0.08);
			pwm4->set(0.08);
			pwm5->set(0.08);
			
			wd->set(true);
			
			homeSensorRad1->get();
			homeSensorRad2->get();
			homeSensorRad3->get();
			
			taster1->get();
			taster2->get();
			taster3->get();
			taster4->get();
			
			led1->set(true);
			led2->set(true);
			led3->set(true);
			led4->set(true);

			if(taster1->get()) {
				stop();
			}
		}
			
// 		void run() {
			
// 			enc0.run();
// 			enc1.run();
// 			enc2.run();
// 			enc3.run();
// 			enc4.run();
// 			enc5.run();
// 			pwm0.getIn().getSignal().setValue(0.08);
// 			pwm1.getIn().getSignal().setValue(0.08);
// 			pwm2.getIn().getSignal().setValue(0.08);
// 			pwm3.getIn().getSignal().setValue(0.08);
// 			pwm4.getIn().getSignal().setValue(0.08);
// 			pwm5.getIn().getSignal().setValue(0.08);
// 			wd.getIn().getSignal().setValue(true);          // gehen nicht
// 			home1.run();
// 			home2.run();                    
// 			home3.run();                    
// 			taster1.run();                  
// 			taster2.run();
// 			taster3.run();
// 			taster4.run();
			
// 			led1.getIn().getSignal().setValue(false); // geht nicht muss an etwas angeschlossen sein
// 			led2.getIn().getSignal().setValue(true);
// 			led3.getIn().getSignal().setValue(false);
// 			led4.getIn().getSignal().setValue(true);
// 			hall0.run();
// 			hall1.run();
// 			hall2.run();
// 			hall3.run();
// 			accy.run();
// 			accx.run();
// 			accz.run();

// 			if(HAL::getLogicPeripheralInput("t1")->get()) {
// 				stop();
// 			}
			
// 			if(taster1.getOut().getSignal().getValue()) { 
// 				stop();
// 			}
// 		}
		
	private:
		
		eeros::hal::PeripheralInput<double>* enc0;
		eeros::hal::PeripheralInput<double>* enc1;
		eeros::hal::PeripheralInput<double>* enc2;
		eeros::hal::PeripheralInput<double>* enc3;
		eeros::hal::PeripheralInput<double>* enc4;
		eeros::hal::PeripheralInput<double>* enc5;

		eeros::hal::PeripheralOutput<double>* pwm0;
		eeros::hal::PeripheralOutput<double>* pwm1;
		eeros::hal::PeripheralOutput<double>* pwm2;
		eeros::hal::PeripheralOutput<double>* pwm3;
		eeros::hal::PeripheralOutput<double>* pwm4;
		eeros::hal::PeripheralOutput<double>* pwm5;
		
		eeros::hal::PeripheralOutput<bool>* wd;        // geht nicht
		
		eeros::hal::PeripheralInput<bool>* homeSensorRad1;
		eeros::hal::PeripheralInput<bool>* homeSensorRad2;
		eeros::hal::PeripheralInput<bool>* homeSensorRad3;
		
		eeros::hal::PeripheralInput<bool>* taster1;
		eeros::hal::PeripheralInput<bool>* taster2; 	
		eeros::hal::PeripheralInput<bool>* taster3;		
		eeros::hal::PeripheralInput<bool>* taster4;
		
		eeros::hal::PeripheralOutput<bool>* led1;
		eeros::hal::PeripheralOutput<bool>* led2;
		eeros::hal::PeripheralOutput<bool>* led3;
		eeros::hal::PeripheralOutput<bool>* led4;
		
		
		
// 		eeros::control::Constant<bool> constantTrue;
		
// 		eeros::control::PeripheralInput<double> enc0;
// 		eeros::control::PeripheralInput<double> enc1;
// 		eeros::control::PeripheralInput<double> enc2;
// 		eeros::control::PeripheralInput<double> enc3;
// 		eeros::control::PeripheralInput<double> enc4;
// 		eeros::control::PeripheralInput<double> enc5;
	
// 		eeros::control::PeripheralOutput<double> pwm0;
// 		eeros::control::PeripheralOutput<double> pwm1;
// 		eeros::control::PeripheralOutput<double> pwm2;
// 		eeros::control::PeripheralOutput<double> pwm3;
// 		eeros::control::PeripheralOutput<double> pwm4;
// 		eeros::control::PeripheralOutput<double> pwm5;
		
// 		eeros::control::PeripheralOutput<bool> wd;
		
// 		eeros::control::PeripheralInput<bool> home1;
// 		eeros::control::PeripheralInput<bool> home2;
// 		eeros::control::PeripheralInput<bool> home3;
		
		
// 		eeros::control::PeripheralInput<bool> taster1;
// 		eeros::control::PeripheralInput<bool> taster2;
// 		eeros::control::PeripheralInput<bool> taster3;
// 		eeros::control::PeripheralInput<bool> taster4;
		
// 		eeros::control::PeripheralOutput<bool> led1;
// 		eeros::control::PeripheralOutput<bool> led2;
// 		eeros::control::PeripheralOutput<bool> led3;
// 		eeros::control::PeripheralOutput<bool> led4;
		
// 		eeros::control::PeripheralInput<double> hall0;
// 		eeros::control::PeripheralInput<double> hall1;
// 		eeros::control::PeripheralInput<double> hall2;
// 		eeros::control::PeripheralInput<double> hall3;
// 		eeros::control::PeripheralInput<double> accy;
// 		eeros::control::PeripheralInput<double> accx;
// 		eeros::control::PeripheralInput<double> accz;
		
};



int main(int argc, char *argv[]) 
{
	
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	w.show(~0); // show all messages
	Logger<LogWriter> log;
	
	// Start
	log.info() << "Test mainboard started...";
	
	// Initialize hardware
	log.info() << "Initializing hardware";
	
	HAL& hal = HAL::instance();
	
	log.trace() << "  Creating device structure...";
	FlinkDevice onBoard (ON_BOAD_FPGA_DEVICE);
	
	// Encoder
	FlinkFqd enc_0("q0", &onBoard, ON_BOARD_FQD_ID, 0, 6.28318530718 / (4 * 1024*18), 0); // 36.75 ist die Getriebe übersetzung Lenkung, 18:antrieb
	FlinkFqd enc_1("q1", &onBoard, ON_BOARD_FQD_ID, 1, 6.28318530718 / (4 * 1024*18), 0);
	FlinkFqd enc_2("q2", &onBoard, ON_BOARD_FQD_ID, 2, 6.28318530718 / (4 * 1024*18), 0);
	FlinkFqd enc_3("q3", &onBoard, ON_BOARD_FQD_ID, 3, 6.28318530718 / (4 * 1024*36.75), 0);
	FlinkFqd enc_4("q4", &onBoard, ON_BOARD_FQD_ID, 4, 6.28318530718 / (4 * 1024*36.75), 0);
	FlinkFqd enc_5("q5", &onBoard, ON_BOARD_FQD_ID, 5, 6.28318530718 / (4 * 1024*36.75), 0);
	
	// PWM
	FlinkPwm pwmaxis0("pwmAxis0",&onBoard, ON_BOARD_PWM_ID, 0);
	FlinkPwm pwmaxis1("pwmAxis1",&onBoard, ON_BOARD_PWM_ID, 1);
	FlinkPwm pwmaxis2("pwmAxis2",&onBoard, ON_BOARD_PWM_ID, 2);
	FlinkPwm pwmaxis3("pwmAxis3",&onBoard, ON_BOARD_PWM_ID, 3);
	FlinkPwm pwmaxis4("pwmAxis4",&onBoard, ON_BOARD_PWM_ID, 4);
	FlinkPwm pwmaxis5("pwmAxis5",&onBoard, ON_BOARD_PWM_ID, 5);
// 	FlinkPwm pwmBaxis0("pwmBaxis0",&onBoard, 1, 0);
// 	FlinkPwm pwmBaxis1("pwmBaxis1",&onBoard, 1, 1);
// 	FlinkPwm pwmBaxis2("pwmBaxis2",&onBoard, 1, 2);
// 	FlinkPwm pwmBaxis3("pwmBaxis3",&onBoard, 1, 3);
// 	FlinkPwm pwmBaxis4("pwmBaxis4",&onBoard, 1, 4);
// 	FlinkPwm pwmBaxis5("pwmBaxis5",&onBoard, 1, 5);
	
	// Frequenz von pwm setzen
	pwmaxis0.setFrequency(25000);
	pwmaxis1.setFrequency(25000);
	pwmaxis2.setFrequency(25000);
	pwmaxis3.setFrequency(25000);
	pwmaxis4.setFrequency(25000);
	pwmaxis5.setFrequency(25000);
	
	// Watchdog
	FlinkWatchdog watchdog("watchd", &onBoard, ON_BOARD_WATCHDOG_ID,0.008); // [s]// braucht min. alle 0.009 s
	
	// homing sensor
	FlinkDigIn home1("rad1", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD1); // zum Invertieren letzten Parameter auf true setzen, Zahl ist welcher Sensorbenutzt wird
	FlinkDigIn home2("rad2", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD2);
	FlinkDigIn home3("rad3", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD3);
	
	// taster 
	FlinkDigIn taster1("t1", &onBoard, ON_BOARD_IN_IO_ID, 0);
	FlinkDigIn taster2("t2", &onBoard, ON_BOARD_IN_IO_ID, 1);
	FlinkDigIn taster3("t3", &onBoard, ON_BOARD_IN_IO_ID, 3);
	FlinkDigIn taster4("t4", &onBoard, ON_BOARD_IN_IO_ID, 2);
	
	FlinkDigOut ledT1("led1", &onBoard, ON_BOARD_OUT_IO_ID, 9);
	FlinkDigOut ledT2("led2", &onBoard, ON_BOARD_OUT_IO_ID, 8);
	FlinkDigOut ledT3("led3", &onBoard, ON_BOARD_OUT_IO_ID, 10);
	FlinkDigOut ledT4("led4", &onBoard, ON_BOARD_OUT_IO_ID, 11);
	
	
    // hall sensoren
// 	FlinkAnalogIn hall0("hall_0", &onBoard, SENSOR_PRINT_ADC_ID,0);
// 	FlinkAnalogIn hall1("hall_1", &onBoard, SENSOR_PRINT_ADC_ID,1);
// 	FlinkAnalogIn hall2("hall_2", &onBoard, SENSOR_PRINT_ADC_ID,2);
// 	FlinkAnalogIn hall3("hall_3", &onBoard, SENSOR_PRINT_ADC_ID,3);
// 	FlinkAnalogIn accy("acc_y",   &onBoard, SENSOR_PRINT_ADC_ID,4);
// 	FlinkAnalogIn accx("acc_x",   &onBoard, SENSOR_PRINT_ADC_ID,5);
// 	FlinkAnalogIn accz("acc_z",   &onBoard, SENSOR_PRINT_ADC_ID,6);

	
	// Im Hal dazu fuegen
	hal.addPeripheralInput(&enc_0);
	hal.addPeripheralInput(&enc_1);
	hal.addPeripheralInput(&enc_2);
	hal.addPeripheralInput(&enc_3);
	hal.addPeripheralInput(&enc_4);
	hal.addPeripheralInput(&enc_5);
	
	hal.addPeripheralOutput(&pwmaxis0);
	hal.addPeripheralOutput(&pwmaxis1);
	hal.addPeripheralOutput(&pwmaxis2);
	hal.addPeripheralOutput(&pwmaxis3);
	hal.addPeripheralOutput(&pwmaxis4);
	hal.addPeripheralOutput(&pwmaxis5);

	hal.addPeripheralOutput(&watchdog);
	
	hal.addPeripheralInput(&home1);
	hal.addPeripheralInput(&home2);
	hal.addPeripheralInput(&home3);
	
	hal.addPeripheralInput(&taster1);
	hal.addPeripheralInput(&taster2);
	hal.addPeripheralInput(&taster3);
	hal.addPeripheralInput(&taster4);
	hal.addPeripheralOutput(&ledT1);
	hal.addPeripheralOutput(&ledT2);
	hal.addPeripheralOutput(&ledT3);
	hal.addPeripheralOutput(&ledT4);
	
// 	hal.addPeripheralInput(&hall0); 
// 	hal.addPeripheralInput(&hall1);
// 	hal.addPeripheralInput(&hall2);
// 	hal.addPeripheralInput(&hall3);
// 	hal.addPeripheralInput(&accy);
// 	hal.addPeripheralInput(&accx);
// 	hal.addPeripheralInput(&accz);
	
	double periodTime = 0.001; // [s] weniger wie 0.008 s (wegen wd) er braucht alle 18ms eine neue flanke
	
	log.info() << "creating test thread";
	Worker test(periodTime);
	
	sleep(1);
		

	log.info() << "starting test thread";
	watchdog.reset();
	test.start();
	
	log.info() << "waiting for test thread";
// 	sleep(10);
// 	test.stop();
	test.join(); // Wartet solange bis der Thred fertig
	
	log.info() << "test thread finished";
    
	ledT1.set(false);
	ledT2.set(false);
	ledT3.set(false);
	ledT4.set(false);
		
	log.trace() << "Test mainboard ended";
    return 0;
	
}