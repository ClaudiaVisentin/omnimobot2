#include <eeros/hal/HAL.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/FlinkDevice.hpp>
#include <eeros/hal/FlinkFqd.hpp>
#include <eeros/hal/FlinkPwm.hpp>
#include <eeros/hal/FlinkWatchdog.hpp>
#include <eeros/core/PeriodicThread.hpp>
#include <eeros/hal/FlinkDigIn.hpp>

#include <signal.h>
#include <iostream>
#include <unistd.h>

#define ON_BOAD_FPGA_DEVICE "/dev/flink0"
#define ON_BOARD_WATCHDOG_ID 1
#define ON_BOARD_IN_IO_ID 7


#define MOTOR_BRIDGE_DEVICE "/dev/flink1"
#define MOTOR_BRIDGE_PWM_A_ID 0 // alle in die eine richtung drehen (motoren)
#define MOTOR_BRIDGE_PWM_B_ID 1
#define MOTOR_BRIDGE_FQD_ID 2

#define HOMSENSOR_RAD1 14


using namespace eeros;
using namespace eeros::logger;
using namespace eeros::hal;



class SPItest : public PeriodicThread {
	public:
		SPItest(double period,FlinkFqd& enc0,FlinkFqd& enc1,FlinkFqd& enc2,FlinkFqd& enc3,FlinkFqd& enc4,FlinkFqd& enc5,FlinkPwm& pwmAax0,FlinkPwm& pwmAax1,FlinkPwm& pwmAax2,
		FlinkPwm& pwmAax3,FlinkPwm& pwmAax4,FlinkPwm& pwmAax5,FlinkPwm& pwmBax0,FlinkPwm& pwmBax1,FlinkPwm& pwmBax2,FlinkPwm& pwmBax3,FlinkPwm& pwmBax4,
		FlinkPwm& pwmBax5,FlinkDigIn& sensor, FlinkWatchdog& wd) : 
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
		
		sensor(sensor),
		
		wd(wd),
		PeriodicThread(period, 0, true, paused){

		}
			
		void run() {
			wd.set(true);
			
			
			pwmAax0.setDutyCycle(0.0);
			pwmAax1.setDutyCycle(0.0);
			pwmAax2.setDutyCycle(0.0);
			pwmAax3.setDutyCycle(0.0);
			pwmAax4.setDutyCycle(0.0);
			pwmAax5.setDutyCycle(0.0);
			
			pwmBax0.setDutyCycle(0.08);
			pwmBax1.setDutyCycle(0.08);
			pwmBax2.setDutyCycle(0.08);
			pwmBax3.setDutyCycle(0.0);
			pwmBax4.setDutyCycle(0.0);
			pwmBax5.setDutyCycle(0.0);
		
			enc0.get();
			enc1.get();
			enc2.get();
			enc3.get();
			enc4.get();
			enc5.get();
			
// 			log.info() << enc2.get();
			
			
			
			
			if(sensor.get()) { // Endschalter erreicht
				stop();
			}
		}
		
		
	private:
			
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
		
		FlinkWatchdog& wd;
		
		FlinkDigIn& sensor;
		
		double angle;	
		int counter;	
		int i;	

};



int main(int argc, char *argv[]) 
{
	
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	w.show(~0); // show all messages
	//w.show(3); // show first four levels
	Logger<LogWriter> log;
	
	// Start
	log.info() << "Test SPI started...";
	
	// Initialize hardware
	log.info() << "Initializing hardware";
	
	HAL& hal = HAL::instance();
	
	log.trace() << "  Creating device structure...";
	FlinkDevice motorBridge (MOTOR_BRIDGE_DEVICE);
	FlinkDevice onBoard (ON_BOAD_FPGA_DEVICE);
	
	// Encoder
	FlinkFqd enc_0("q0", &motorBridge, MOTOR_BRIDGE_FQD_ID, 0, 6.28318530718 / (4 * 1024*18), 0); // 36.75 ist die Getriebe übersetzung Lenkung, 18:antrieb
	FlinkFqd enc_1("q1", &motorBridge, MOTOR_BRIDGE_FQD_ID, 1, 6.28318530718 / (4 * 1024*18), 0);
	FlinkFqd enc_2("q2", &motorBridge, MOTOR_BRIDGE_FQD_ID, 2, 6.28318530718 / (4 * 1024*18), 0);
	FlinkFqd enc_3("q3", &motorBridge, MOTOR_BRIDGE_FQD_ID, 3, 6.28318530718 / (4 * 1024*36.75), 0);
	FlinkFqd enc_4("q4", &motorBridge, MOTOR_BRIDGE_FQD_ID, 4, 6.28318530718 / (4 * 1024*36.75), 0);
	FlinkFqd enc_5("q5", &motorBridge, MOTOR_BRIDGE_FQD_ID, 5, 6.28318530718 / (4 * 1024*36.75), 0);
	
	// PWM
	FlinkPwm pwmAaxis0("pwmAaxis0",&motorBridge, 0, 0);
	FlinkPwm pwmAaxis1("pwmAaxis1",&motorBridge, 0, 1);
	FlinkPwm pwmAaxis2("pwmAaxis2",&motorBridge, 0, 2);
	FlinkPwm pwmAaxis3("pwmAaxis3",&motorBridge, 0, 3);
	FlinkPwm pwmAaxis4("pwmAaxis4",&motorBridge, 0, 4);
	FlinkPwm pwmAaxis5("pwmAaxis5",&motorBridge, 0, 5);
	FlinkPwm pwmBaxis0("pwmBaxis0",&motorBridge, 1, 0);
	FlinkPwm pwmBaxis1("pwmBaxis1",&motorBridge, 1, 1);
	FlinkPwm pwmBaxis2("pwmBaxis2",&motorBridge, 1, 2);
	FlinkPwm pwmBaxis3("pwmBaxis3",&motorBridge, 1, 3);
	FlinkPwm pwmBaxis4("pwmBaxis4",&motorBridge, 1, 4);
	FlinkPwm pwmBaxis5("pwmBaxis5",&motorBridge, 1, 5);
	
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
	
	
	
	// homing sensor
	FlinkDigIn home("home", &onBoard, ON_BOARD_IN_IO_ID, HOMSENSOR_RAD1); // zum Invertieren letzten Parameter auf true setzen, Zahl ist welcher Sensorbenutzt wird
	
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
	
	hal.addPeripheralInput(&home);
	
	//PWM A
	std::cout << "Setup PWM " << std::endl;
	


	
	double periodTime = 0.001; // [s] weniger wie 0.008 s
	
	log.info() << "creating test thread";
	SPItest test(periodTime,enc_0,enc_1,enc_2,enc_3,enc_4,enc_5,pwmAaxis0,pwmAaxis1,pwmAaxis2,pwmAaxis3,pwmAaxis4,pwmAaxis5,pwmBaxis0,pwmBaxis1,pwmBaxis2,pwmBaxis3,pwmBaxis4,pwmBaxis5, home, watchdog	);
	
	sleep(1);
	
	log.info() << "starting test thread";
	watchdog.reset();
	test.start();
	
	log.info() << "waiting for test thread";
	test.join(); // Wartet solange bis der Thred fertig
	
	log.info() << "test thread finished";
	
	// pwm zurücksetzen
	pwmBaxis0.setDutyCycle(0.0);
	pwmBaxis1.setDutyCycle(0.0);
	pwmBaxis2.setDutyCycle(0.0);
	pwmBaxis3.setDutyCycle(0.0);
	pwmBaxis4.setDutyCycle(0.0);
	pwmBaxis5.setDutyCycle(0.0);

	log.trace() << "Test SPI ended";
    return 0;
}