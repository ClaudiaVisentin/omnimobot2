#ifndef __CH_NTB_OMNIMOBOT_SAFTY_OMNISAFETYPROPERTIES_HPP
#define __CH_NTB_OMNIMOBOT_SAFTY_OMNISAFETYPROPERTIES_HPP
#include <omnimobot/Counter.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/FlinkWatchdog.hpp>
#include <eeros/hal/FlinkFqd.hpp>
#include <omnimobot/types.hpp>


namespace omnimobot
{
	class ControlSystem;
	namespace event
	{
		
		// Name all event
		enum
		{
			doLowEmergency = 99,
			doEmergency = 98,
			resetLowEmergency = 1,
			resetEmergency =2,
			doOff =3,				
			doControlStop =4,
			controlStoppingDone =5,

			doInit = 10,
			InitDone = 11,

			homingDoneRad1 = 20,
			homingDoneRad2 = 21,
			homingDoneRad3 = 22,

			doPowerOn = 30,
			PoweringDone = 31,

			doDriveJoystick = 40,
			doDriveAutonom = 41,
			doDriveJoystickPendulum = 42,
			doDriveAutonomPendulum = 43,
			
			stop = 50,
			
			doShutDown = 100
		};
	}

	namespace level
	{
		// Define all possible level
		enum
		{
			off = 100,
			initializing = 110,
			
			initialized = 200,
			lowEmergency = 210,
			powerOn = 220,
			
			homingRad1 = 300,
			homingRad2 = 310,
			homingRad3 = 320,

			emergency = 400,
			ready = 410,
			driveJoystick = 420,
			driveAutonom = 430,
			driveJoystickPendulum = 440,
			driveAutonomPendulum = 450,
			
			controlStopping = 500,
			
			shutDown = 600,
		};
	}


class OmniSafetyProperties : public eeros::safety::SafetyProperties {
	public:
		OmniSafetyProperties(ControlSystem* cs, eeros::hal::FlinkWatchdog& motorBoardWatchdog);
		virtual ~OmniSafetyProperties();
		
		// critical Outputs
		eeros::hal::PeripheralOutput<double>* pwmA0;
		eeros::hal::PeripheralOutput<double>* pwmA1;
		eeros::hal::PeripheralOutput<double>* pwmA2;
		eeros::hal::PeripheralOutput<double>* pwmA3;
		eeros::hal::PeripheralOutput<double>* pwmA4;
		eeros::hal::PeripheralOutput<double>* pwmA5;
		eeros::hal::PeripheralOutput<double>* pwmB0;
		eeros::hal::PeripheralOutput<double>* pwmB1;
		eeros::hal::PeripheralOutput<double>* pwmB2;
		eeros::hal::PeripheralOutput<double>* pwmB3;
		eeros::hal::PeripheralOutput<double>* pwmB4;
		eeros::hal::PeripheralOutput<double>* pwmB5;
		
		eeros::hal::PeripheralOutput<bool>* emergencyLED;
		eeros::hal::PeripheralOutput<bool>* powerOnLED;
		
		// critical inputs
		eeros::hal::PeripheralInput<bool>* emergencyReset; 		// taster
		eeros::hal::PeripheralInput<bool>* approval;			// taster
		eeros::hal::PeripheralInput<bool>* emergencyStop;		// safety circuit is active

		// not critical Inputs and Outputs
		eeros::hal::PeripheralInput<bool>* approvalPendulum;	// taster
		eeros::hal::PeripheralInput<bool>* driveStop;			// taster
		
		eeros::hal::PeripheralOutput<bool>* approvalLED;
		eeros::hal::PeripheralOutput<bool>* emergencyResetLED;
		eeros::hal::PeripheralOutput<bool>* stopDriveLED;

		eeros::hal::PeripheralInput<bool>* homeSensorWheel1;
		eeros::hal::PeripheralInput<bool>* homeSensorWheel2;
		eeros::hal::PeripheralInput<bool>* homeSensorWheel3;


	private:
		ControlSystem* controlSys;
		eeros::hal::FlinkWatchdog& watchdog;
		eeros::math::Matrix<6,1> encOffset;
		Counter counter;
	};

}

#endif /* __CH_NTB_OMNIMOBOT_SAFTY_OMNISAFETYPROPERTIES_HPP */
