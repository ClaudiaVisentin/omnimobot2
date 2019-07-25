#ifndef __CH_NTB_OMNIMOBOT_SAFETYWITHOUTCONTROLSYSTEM_HPP
#define __CH_NTB_OMNIMOBOT_SAFETYWITHOUTCONTROLSYSTEM_HPP

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/SafetyProperties.hpp>
//#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>
//#include <eeros/hal/ScalablePeripheralInput.hpp>
#include <eeros/hal/FlinkWatchdog.hpp>
#include <eeros/hal/FlinkFqd.hpp>
#include <types.hpp>


namespace omnimobot
{
	class ControlsystemDummy;
	namespace event
	{
		
		// Name all event
		enum
		{
			doLowEmergency = 99,
			doEmergency = 98,
			resetLowEmergency = 1,
			resetEmergency =2,
			doOff =3,				// gleichzeitig wie sequenzer (main wird beendet
			doControlStopp =4,
			controlStoppingDone =5,

			doInit = 10,
			InitDone = 11,
			approvalOn = 12,

			homingDoneRad1 = 20,
			homingDoneRad2 = 21,
			homingDoneRad3 = 22,

			doPowerOn = 30,
			PoweringDone = 31,

			doDriveJoystick = 40,
			doDriveAutonom = 41,
			stop = 42
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
			
			controlStopping = 500,

		};
	}


	class SafetyWithoutControlSystem : public eeros::safety::SafetyProperties {
		public:
			SafetyWithoutControlSystem(ControlsystemDummy* cs, eeros::hal::FlinkWatchdog* motorBoardWatchdog);
			virtual ~SafetyWithoutControlSystem();
			
			// critical outputs
			eeros::hal::PeripheralOutput<bool>* watchdog;
			
			eeros::hal::PeripheralOutput<bool>* power;
			
		
			
			// critical inputs
			eeros::hal::PeripheralInput<bool>* piltz;
			eeros::hal::PeripheralInput<bool>* emergencyStop; 		// um emergency zu reseten
			eeros::hal::PeripheralInput<bool>* approval;			// Taster

			
			// not critical Inputs and Outputs
			
			eeros::hal::PeripheralOutput<bool>* approvalLED;
			eeros::hal::PeripheralOutput<bool>* emergencyLED;
			eeros::hal::PeripheralOutput<bool>* emergencyStop_LED;
			eeros::hal::PeripheralOutput<bool>* powerOnLED;


		
		private:
			ControlsystemDummy* controlSys;
			eeros::hal::FlinkWatchdog* watchdogDevice;
		};
}

#endif /* __CH_NTB_OMNIMOBOT_SAFETYWITHOUTCONTROLSYSTEM_HPP */
