#include <eeros/hal/HAL.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/FlinkDevice.hpp>
#include <eeros/hal/FlinkPwm.hpp>
#include <eeros/hal/FlinkFqd.hpp>
#include <eeros/core/PeriodicThread.hpp>
#include <eeros/hal/FlinkDigOut.hpp>

#include <math.h>       /* fabs */

#include <signal.h>
#include <iostream>
#include <unistd.h>

#define ON_BOAD_FPGA_DEVICE "/dev/flink0"
#define ON_BOARD_PWM_ID 0
#define ON_BOARD_FQD_ID 5
#define ON_BOARD_OUT_IO_ID 6

#define MOTOR_BRIDGE_DEVICE "/dev/flink1"
#define MOTOR_BRIDGE_FQD_ID 2

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::hal;

class maxonPrintTest : public PeriodicThread {
	public:
		maxonPrintTest(double gear, double desiredpos, double period, double inert, FlinkPwm& pwmax, FlinkFqd& enc,FlinkDigOut& change ):
		PeriodicThread(period, 0, true, paused),
		pwm(pwmax),
		q(enc),
		directionChange(change),
		errorPos(0.0),
		errorSpeed(0.0),
		desPos(desiredpos),
		desVelo(0.0),
		settingSpeed(0.0),
		actualSpeed(0.0),
		actualPos(0.0),
		oldPos(0.0),
		dt(period),
		acceleration(0.0),
		inertia(inert),
		gear(gear), // gear antrieb=18, gear lenkung =36.75
		voltage(0.0),
 		dutyCycle(0.0),
 		olddesPos(0.0),
 		desiredPos(0.0),
 		current(0.0),
 		ended(false),
 		tmp(0) {
		}
		
		
		void run(){
			actualPos = q.get();
			
			// ramp desired position
			if(desPos > olddesPos){
				desiredPos = olddesPos + 0.005;
			}

			desVelo = (desiredPos - olddesPos)/dt;
			
			actualSpeed = (actualPos - oldPos)/dt;
			
			errorPos = desiredPos - actualPos;

			
			settingSpeed = desVelo + errorPos * 255.102/2.0; // see Kp in doku (kaskadierte Regelung) w/(2D), kp = 255.102
			

			// Velocity saturation
			if(settingSpeed > 790.0){
				settingSpeed = 790.0;
			}
			else if(settingSpeed < -790.0){
				settingSpeed = -790.0;
			}
			
			errorSpeed = settingSpeed - actualSpeed;
			
			acceleration = errorSpeed * 500.0/2.0;  // see Kv in doku (kaskadierte Regelung) w2D, kv = 500.0
			
			current = acceleration * inertia * 1.0/gear * 1.0/0.0302; // 0.0302 = km
			
			
			// current saturation
			if(current > 15.0){
				current = 15.0;
			}
			else if(current < -15.0){
				current = -15.0;
			}
		
			//voltage = current * 0.299 + actualSpeed * gear * 0.0302; // 0.0302 = km, 0.299 = R
			

			dutyCycle = (fabs(current) / 15.0);// 15 is max current

			// dutyCycle saturation
			if(dutyCycle < 0.1){		// anpassen auf Soll-Wert vorgabe in ESCON
				dutyCycle = 0.1;
			}
			
			if(dutyCycle > 0.89){		// anpassen auf Soll-Wert vorgabe in ESCON
				dutyCycle = 0.89;
			}
			
			// direction
			if(current >= 0.0){
				directionChange.set(false);
			}
			else {
				directionChange.set(true);
			}
			
			// set pwm
 			if(ended){
				pwm.setDutyCycle(0.10);
			}
			else{
				pwm.setDutyCycle(dutyCycle);
			}
			
			oldPos = actualPos;
			olddesPos = desiredPos;
			
			if(tmp == 100){
 				log.info() << "enc-Daten: " << q.get() << " dutyCycle: "<< dutyCycle << " richtung " << (current >= 0.0) << " Soll- wert: " << desiredPos << " desVelo " << desVelo << " actualSpeed: " << actualSpeed ;
//				log.info() << "actualPos: " << actualPos << " desiredPos: "<< desiredPos << " desVelo " << desVelo << " actualSpeed: " << actualSpeed ;
				tmp = 0;
			}
			
			tmp = tmp + 1;
		}
		
		double getEnc(){
			return q.get();
		}
		
		double getDesiredPos(){
			return desPos;
		}
		
		void setDesiredPos(double dePos){
			desPos = dePos;
		}
		
		void setEnded(bool end){
			ended = end;
		}
		
	private:
		
		FlinkPwm& pwm;	
		FlinkFqd& q;
		FlinkDigOut& directionChange;
		
		double errorPos;
		double errorSpeed;
		double desPos;
		double desVelo;
		double settingSpeed;
		double actualSpeed;
		double actualPos;
		double oldPos;
		double dt;
		double acceleration;
		double inertia;
		double gear;
		double voltage;
		double dutyCycle;
		double olddesPos;
		double desiredPos;
		double current;
		bool ended;
		
		int tmp;
};


int main(int argc, char *argv[]) 
{
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	w.show(~0); // show all messages
	//w.show(3); // show first four levels
	Logger<LogWriter> log;
	
	// Start
	log.info() << "Test maxon started...";
	
	// Initialize hardware
	log.info() << "Initializing hardware";
	
	HAL& hal = HAL::instance();
	
	log.trace() << "  Creating device structure...";
	FlinkDevice onBoard (ON_BOAD_FPGA_DEVICE);
	FlinkDevice motorBridge (MOTOR_BRIDGE_DEVICE);
	
	// PWM
	FlinkPwm pwmaxis("pwmaxis",&onBoard, ON_BOARD_PWM_ID, 0.0);
	// Encoder
	FlinkFqd enc("q", &onBoard, ON_BOARD_FQD_ID, 7, 6.28318530718 / (4 * 1024*36.75), 0); // 36.75 ist die Getriebe übersetzung Lenkung, 18:antrieb
	// direction change
	FlinkDigOut switchDirection("change", &onBoard, ON_BOARD_OUT_IO_ID, 12);
	
	
	// set frequenz pwm 
	pwmaxis.setFrequency(2000); // min. 2 * period time
	
	hal.addPeripheralOutput(&pwmaxis);
	hal.addPeripheralInput(&enc);
	hal.addPeripheralOutput(&switchDirection);
	
	double periodTime = 0.001; // [s] lower than 0.008 s for WD
	
	double desiredPos = 0.2; // desired position
	
	double inert = 0.029557; // 0.011962 (Antrieb), 0.029557 static inertia (Lenkung) [kg*m²]
	double gear = 36.75; //gear antrieb=18, gear lenkung =36.75
	
	log.info() << "creating test thread";
	maxonPrintTest test(gear, desiredPos, periodTime,inert, pwmaxis, enc,switchDirection);
	
	test.start();
	
	sleep(10);
		
	test.setEnded(true);
	
	sleep(1);
	
	test.stop();
	
	// return pwm 
	pwmaxis.setDutyCycle(0.10);
	
	
	log.trace() << "Test maxon ended";
    return 0;
}