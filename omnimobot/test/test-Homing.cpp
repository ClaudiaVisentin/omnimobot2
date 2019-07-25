#include <omnimobot/control/ControlSystem.hpp>
#include <eeros/hal/HAL.hpp>

#include <eeros/hal/FlinkDevice.hpp>
#include <eeros/hal/FlinkDigIn.hpp>
#include <eeros/hal/FlinkFqd.hpp>
#include <eeros/hal/FlinkPwm.hpp>
#include <eeros/hal/FlinkDigOut.hpp>
#include <eeros/core/PeriodicThread.hpp>

#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
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

using namespace omnimobot;
using namespace eeros;
using namespace eeros::control;
using namespace eeros::logger;
using namespace eeros::hal;

class Worker : public PeriodicThread {
	public:
		Worker(double period,FlinkDigIn& taster, FlinkDigOut& taster_LED,FlinkFqd& enc0,FlinkFqd& enc1,FlinkFqd& enc2,FlinkFqd& enc3,FlinkFqd& enc4,FlinkFqd& enc5,FlinkPwm& pwmAax0,FlinkPwm& pwmAax1,FlinkPwm& pwmAax2,
		FlinkPwm& pwmAax3,FlinkPwm& pwmAax4,FlinkPwm& pwmAax5,FlinkPwm& pwmBax0,FlinkPwm& pwmBax1,FlinkPwm& pwmBax2,FlinkPwm& pwmBax3,FlinkPwm& pwmBax4,
		FlinkPwm& pwmBax5,FlinkDigIn& sensor1, FlinkDigIn& sensor2, FlinkDigIn& sensor3):
		taster1(taster),
		taster1_LED(taster_LED),
		enc0(enc0),
		enc1(enc1),
		enc2(enc2),
		enc3(enc3),
		enc4(enc4),
		enc5(enc5),
		pwmAax0(pwmAax0),
		pwmAax1(pwmAax1),
		pwmAax2(pwmAax2),
		pwmAax3(pwmAax3),
		pwmAax4(pwmAax4),
		pwmAax5(pwmAax5),
		pwmBax0(pwmBax0),
		pwmBax1(pwmBax1),
		pwmBax2(pwmBax2),
		pwmBax3(pwmBax3),
		pwmBax4(pwmBax4),
		pwmBax5(pwmBax5),
		sensor1(sensor1),
		sensor2(sensor2),	
		sensor3(sensor3),	
		first(true),
		steerNumber(0),
		firstRad1(true),
		firstRad2(true),
		firstRad3(true),
		controlSys(period),
		PeriodicThread(period, 0, true, paused){
			// craete control system
			log.info() << "create control System";	
			resetValue.zero();
			offset.zero();
			angle.zero();
		}
		
		~Worker(){};
		
		
		void initHoming() {	
			
			controlSys.initHoming();
		}
		
		
		void setWheelToHoming(int rad){
			
			controlSys.setWheelToHoming(rad);
			
			steerNumber = rad;
// 			
// 			if(steerNumber == rad1){	// Homing Rad1
// 				controlSys.robotControlBlock.integrator.setInitCondition(resetValue);
// 				controlSys.robotControlBlock.homingBlock.swSteer1.switchToInput(0); // Rad1 steer get torque 
// 			}
// 			else if(steerNumber == rad2){ // Homing Rad2
// 				controlSys.robotControlBlock.integrator.setInitCondition(resetValue);
// 				controlSys.robotControlBlock.homingBlock.swSteer2.switchToInput(0);
// 			}
// 			else{ // Homing Rad3
// 				controlSys.robotControlBlock.integrator.setInitCondition(resetValue);
// 				controlSys.robotControlBlock.homingBlock.swSteer3.switchToInput(0);
// 			}
		}
		
		
		void run() {
			
			if(first){
				setWheelToHoming(rad1);
				controlSys.start();
				first = false;
			}
			
			// read Data
			angle(rad1) = controlSys.sumOffset.getOut().getSignal().getValue()(rad1);	
			angle(rad2) = controlSys.sumOffset.getOut().getSignal().getValue()(rad2);
			angle(rad3) = controlSys.sumOffset.getOut().getSignal().getValue()(rad3);

			// Errors
			if(steerNumber == rad1){

				if(controlSys.gainDeltaQToQpoint.getOut().getSignal().getValue()(rad1) < 0.0 ){
					controlSys.allAxisStopped();
					controlSys.stop();
					std::cout << "ERROR:1 Homing failed Rad1 "<< std::endl;
					stop();
					return;
				}
			}
			else if(steerNumber == rad2){
								
				if(controlSys.gainDeltaQToQpoint.getOut().getSignal().getValue()(rad2) < 0.0 ){
					controlSys.allAxisStopped();
					controlSys.stop();
					std::cout << "ERROR:2 Homing failed Rad2 " << std::endl; 
					stop();
					return;
				}
			}
			else{

				if(controlSys.gainDeltaQToQpoint.getOut().getSignal().getValue()(rad3) < 0.0){
					controlSys.allAxisStopped();
					controlSys.stop();
					std::cout << "ERROR:3 Homing failed Rad3 "<< std::endl; 
					stop();
					return;
				}			
			}
			
			static int ctr1 = 0;
			static int ctr2 = 0;
			static bool T1LED = true;
			
			// LED-Taster 1 toggeln 
			if ( ctr1 >= 250) {			
				T1LED = !T1LED;
				taster1_LED.set(T1LED);                                
				ctr1 = 0;
				}
				
			ctr1++;
			
			// Output
			if ( ctr2 >= 500) {	
				std::cout<< " angle_rad3: " << angle(rad3) << " angle_rad2: " << angle(rad2) << " angle_rad1: " << angle(rad1) << "PWM A 0: "<< controlSys.voltageToPwm.getOutPwmA().getSignal().getValue()(0)<< std::endl;                              
				ctr2 = 0;
				}
				
			ctr2++;
			
			// Encoder Offset
			if(steerNumber == rad1 && sensor1.get()){
				
				if (firstRad1){
					firstRad1 = false;
					usleep(1000000);
				}
				else {
					controlSys.robotControlBlock.homingBlock.swSteer1.switchToInput(1);	// close output for steer1
					offset(rad1) = -controlSys.integratorQpoint.getOut().getSignal().getValue()(rad1) + 1.776; // Offset Rad1
					offset(rad2) = -controlSys.integratorQpoint.getOut().getSignal().getValue()(rad2);
					controlSys.offsetEnc.setValue(offset);
					usleep(20000);
					angle(rad1) = controlSys.sumOffset.getOut().getSignal().getValue()(rad1);
					std::cout<< " angle_rad3: " << angle(rad3) << " angle_rad2: " << angle(rad2) << " angle_rad1: " << angle(rad1) << std::endl;                              
					setWheelToHoming(rad2);
				}
			}
			else if(steerNumber == rad2 && sensor2.get()){
				
				if (firstRad2){
					firstRad2 = false;
					usleep(1000000);
				}
				else {
					controlSys.robotControlBlock.homingBlock.swSteer2.switchToInput(1);
					offset(rad2) = -controlSys.integratorQpoint.getOut().getSignal().getValue()(rad2) + 2.833; // Offset Rad2
					offset(rad3) = -controlSys.integratorQpoint.getOut().getSignal().getValue()(rad3);
					controlSys.offsetEnc.setValue(offset);
					usleep(20000);
					angle(rad2) = controlSys.sumOffset.getOut().getSignal().getValue()(4);
					std::cout<< " angle_rad3: " << angle(rad3) << " angle_rad2: " << angle(rad2) << " angle_rad1: " << angle(rad1) << std::endl;                              
					setWheelToHoming(rad3);
				}
			}
			else if(sensor3.get()&& steerNumber != rad1 && steerNumber != rad2 ){
				
				if (firstRad3){
					firstRad3 = false;
					usleep(1000000);
				}
				else {
					controlSys.robotControlBlock.homingBlock.swSteer3.switchToInput(1);
					offset(rad3) = -controlSys.integratorQpoint.getOut().getSignal().getValue()(rad3) + 5.6997; // Offset Rad3
					controlSys.offsetEnc.setValue(offset);
					usleep(20000);
					angle(rad3) = controlSys.sumOffset.getOut().getSignal().getValue()(rad3);
					
					std::cout<< " angle_rad3: " << angle(rad3) << " angle_rad2: " << angle(rad2) << " angle_rad1: " << angle(rad1) << std::endl;                              
					log.info() << "Homing successfully";
					
					controlSys.allAxisStopped();
					controlSys.stop();
					stop();	
					return;
				}
			}
			
			
			// Wenn Taster 1 betaetigt, test bricht ab
			if(taster1.get()){
				controlSys.allAxisStopped();
				controlSys.stop();
				
				if(steerNumber == rad1){
					angle(rad1) = controlSys.sumOffset.getOut().getSignal().getValue()(rad1);
				}
				else if(steerNumber == rad2){
					angle(rad2) = controlSys.sumOffset.getOut().getSignal().getValue()(rad2);
				}
				else{
					angle(rad3) = controlSys.sumOffset.getOut().getSignal().getValue()(rad3);
				}
				
				std::cout<< "Taste 1 pressed "<< " angle_rad3: " << angle(rad3) << " angle_rad2: " << angle(rad2) << " angle_rad1: " << angle(rad1) << std::endl;                              
				stop();	
				return;
			}
		}
		
		Vector6 getAngle() {
			return angle;
		}
		
	private:
		
		Vector6 resetValue;
		Vector6 offset;
		Vector6 angle;
		
		FlinkDigIn& taster1;
		FlinkDigOut& taster1_LED;
		
		FlinkFqd& enc0;
		FlinkFqd& enc1;	
		FlinkFqd& enc2;	
		FlinkFqd& enc3;	
		FlinkFqd& enc4;	
		FlinkFqd& enc5;	
			
		FlinkPwm& pwmAax0;	
		FlinkPwm& pwmAax1;	
		FlinkPwm& pwmAax2;	
		FlinkPwm& pwmAax3;	
		FlinkPwm& pwmAax4;	
		FlinkPwm& pwmAax5;	
		FlinkPwm& pwmBax0;	
		FlinkPwm& pwmBax1;	
		FlinkPwm& pwmBax2;	
		FlinkPwm& pwmBax3;	
		FlinkPwm& pwmBax4;	
		FlinkPwm& pwmBax5;	
		
		FlinkDigIn& sensor1;
		FlinkDigIn& sensor2;
		FlinkDigIn& sensor3;
		
		bool first;
		int steerNumber;
		
		bool firstRad1;
		bool firstRad2;
		bool firstRad3;
		
		static constexpr int rad1 = 3;
		static constexpr int rad2 = 4;
		static constexpr int rad3 = 5;
		
		ControlSystem controlSys;

};






int main(int argc, char *argv[])
{
	double periodTime = 0.001; //[s]
	
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	w.show(~0); // show all messages
	//w.show(3); // show first four levels
	Logger<LogWriter> log;
	
	// Start
	log.info() << "Application Homing started...";
	
	// Initialize hardware
	log.info() << "Initializing hardware";
	HAL& hal = HAL::instance();
	
	log.trace() << "  Creating device structure...";
	FlinkDevice onBoard (ON_BOAD_FPGA_DEVICE);
	
	// HMI
	FlinkDigIn taster1("T1", &onBoard, ON_BOARD_IN_IO_ID, 2); 
	FlinkDigOut taster1_LED("T1LED", &onBoard, ON_BOARD_OUT_IO_ID, 1);

	// Homingsensoren
	FlinkDigIn homeSensor1("Rad1", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD1);
	FlinkDigIn homeSensor2("Rad2", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD2);
	FlinkDigIn homeSensor3("Rad3", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD3);
	
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
	
	// Encoder
	FlinkFqd enc_0("q0", &onBoard, ON_BOARD_FQD_ID, 0, 6.28318530718 / (4 * 1024*18), 0,true); // 36.75 ist die Getriebe übersetzung Lenkung, 18:antrieb
	FlinkFqd enc_1("q1", &onBoard, ON_BOARD_FQD_ID, 1, 6.28318530718 / (4 * 1024*18), 0,true); // true: delta q wird ausgegeben
	FlinkFqd enc_2("q2", &onBoard, ON_BOARD_FQD_ID, 2, 6.28318530718 / (4 * 1024*18), 0,true);
	FlinkFqd enc_3("q3", &onBoard, ON_BOARD_FQD_ID, 3, 6.28318530718 / (4 * 1024*36.75), 0,true);
	FlinkFqd enc_4("q4", &onBoard, ON_BOARD_FQD_ID, 4, 6.28318530718 / (4 * 1024*36.75), 0,true);
	FlinkFqd enc_5("q5", &onBoard, ON_BOARD_FQD_ID, 5, 6.28318530718 / (4 * 1024*36.75), 0,true);
	
	// Im Hal dazu fuegen
	hal.addPeripheralInput(&taster1);
	hal.addPeripheralOutput(&taster1_LED);
	hal.addPeripheralInput(&homeSensor1);
	hal.addPeripheralInput(&homeSensor2);
	hal.addPeripheralInput(&homeSensor3);
	
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
	
	hal.addPeripheralInput(&enc_0);
	hal.addPeripheralInput(&enc_1);
	hal.addPeripheralInput(&enc_2);
	hal.addPeripheralInput(&enc_3);
	hal.addPeripheralInput(&enc_4);
	hal.addPeripheralInput(&enc_5);
	

	Worker test(periodTime,taster1,taster1_LED,enc_0,enc_1,enc_2,enc_3,enc_4,enc_5,pwmAaxis0,pwmAaxis1,pwmAaxis2,pwmAaxis3,pwmAaxis4,pwmAaxis5,pwmBaxis0,pwmBaxis1,pwmBaxis2,pwmBaxis3,pwmBaxis4,pwmBaxis5, homeSensor1,homeSensor2,homeSensor3);
	
	sleep(1);
	
	test.initHoming();
	
	log.info() << "starting test thread";
	test.start();
	
	log.info() << "waiting for test thread";
	test.join(); // wait for the stop
	
	log.info() << "test thread finished";
	
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

	log.trace() << "Test Homing ended";

	return 0;
}