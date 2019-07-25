#include "SafetyWithoutControlSystem.hpp"
#include "ControlsystemDummy.hpp"



#include <eeros/hal/HAL.hpp>
#include <eeros/safety/inputActions.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <vector>
#include <initializer_list>

#include <unistd.h>
#include <iostream>
//#include <bits/mathcalls.h>

#define MOTOR_BRIDGE_DEVICE "/dev/flink1"

using namespace omnimobot;

using namespace eeros::logger;

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;

SafetyWithoutControlSystem::SafetyWithoutControlSystem(ControlsystemDummy* cs, FlinkWatchdog* motorBoardWatchdog) :  controlSys(cs), watchdogDevice(motorBoardWatchdog)
{ 
	if(controlSys == nullptr) {
		throw -325; // TODO
	}
	if(motorBoardWatchdog == nullptr) {
		throw -326; // TODO
	}
	
	HAL& hal = HAL::instance();
	
	// ############ Define critical outputs ############
	watchdog = hal.getLogicPeripheralOutput("watchdog");
		
	power = hal.getLogicPeripheralOutput("powerOnLED");		// könnte ein LED sein
	
	
	criticalOutputs = { 
		watchdog, power };

	
	// ############ Define critical inputs ############
	emergencyStop = hal.getLogicPeripheralInput("emergencyStop");
	approval = hal.getLogicPeripheralInput("approval");
	piltz = hal.getLogicPeripheralInput("emergency");
	//laserscanner = hal.getRealPeripheralInput("laserscanner");
	
	criticalInputs = { emergencyStop, approval, piltz};
	
	
	// ############ Define inputs and Outputs ###########
	
	approvalLED =   hal.getLogicPeripheralOutput("approvalLED");
	emergencyLED =  	hal.getLogicPeripheralOutput("emrgencyLED");
	emergencyStop_LED =	hal.getLogicPeripheralOutput("emergencyStop_LED");
	powerOnLED =    hal.getLogicPeripheralOutput("powerOnLED");
	


	// ############ Define Levels ############
	levels =
	{
		{ level::off, 			    "Off state", 												 },       
		{ level::initializing,      "software is initializing",               						 },
		{ level::initialized,       "software is initialized / without homing waiting for approval", },
		{ level::lowEmergency,      "low level emergency state",              					     },
		{ level::controlStopping, 	"Stopping control system", 												 }, 
		{ level::powerOn,           "Power on, controller on, homing starting", 					 }, 
		{ level::homingRad1,        "homing steering 1",                        					 },
		{ level::homingRad2,        "homing steering 2",                      						 },
		{ level::homingRad3,        "homing steering 3",                       						 },
		{ level::emergency,         "emergency state",                          					 },
		{ level::ready,             "Robot is ready",                           					 },
		{ level::driveJoystick,     "moving with joystick",                     					 },
		{ level::driveAutonom,      "Robot is moving autonomous",             						 }
	};
	
	// ############ Add events to the levels ############
	entryLevel = level::off; // Das erste Level nach dem Start

	level( level::off               ).addEvent( event::doInit,                level::initializing,          eeros::safety::kPublicEvent);
	
	level( level::initializing      ).addEvent( event::InitDone,              level::initialized,           eeros::safety::kPublicEvent);
	level( level::initialized       ).addEvent( event::doPowerOn,             level::powerOn,               eeros::safety::kPublicEvent);
	level( level::initialized       ).addEvent( event::doOff,                 level::off,                   eeros::safety::kPublicEvent);
	
	level( level::controlStopping   ).addEvent( event::controlStoppingDone,   level::initialized,          eeros::safety::kPublicEvent);
	
	level( level::powerOn           ).addEvent( event::PoweringDone,          level::homingRad1,            eeros::safety::kPublicEvent);
	level( level::homingRad1        ).addEvent( event::homingDoneRad1,        level::homingRad2,            eeros::safety::kPublicEvent);
	level( level::homingRad2        ).addEvent( event::homingDoneRad2,        level::homingRad3,            eeros::safety::kPublicEvent);
	level( level::homingRad3        ).addEvent( event::homingDoneRad3,        level::ready,                 eeros::safety::kPublicEvent);

	level( level::ready             ).addEvent( event::doDriveJoystick,       level::driveJoystick,         eeros::safety::kPublicEvent);
	level( level::ready             ).addEvent( event::doDriveAutonom,        level::driveAutonom,          eeros::safety::kPublicEvent);
	level( level::ready             ).addEvent( event::doControlStopp,        level::controlStopping,           eeros::safety::kPublicEvent);
	
	level( level::driveJoystick     ).addEvent( event::stop,                  level::ready,                 eeros::safety::kPublicEvent);
	level( level::driveAutonom      ).addEvent( event::stop,                  level::ready,                 eeros::safety::kPublicEvent);
	
	level( level::lowEmergency      ).addEvent( event::doControlStopp,        level::controlStopping,               eeros::safety::kPublicEvent);    
	level( level::lowEmergency      ).addEvent( event::resetLowEmergency,     level::controlStopping,               eeros::safety::kPublicEvent);    
	level( level::emergency         ).addEvent( event::doControlStopp,        level::controlStopping,                 eeros::safety::kPublicEvent);
	level( level::emergency         ).addEvent( event::resetEmergency,        level::ready,                 eeros::safety::kPublicEvent);

	
	
	addEventToAllLevelsBetween(level::powerOn, level::homingRad3, event::doEmergency, level::lowEmergency, eeros::safety::kPublicEvent); // Ein Event zu einem Levelbereich hinzufügen
	addEventToLevelAndAbove(level::ready, event::doEmergency, level::emergency, eeros::safety::kPublicEvent); // ab hier allen ein Event hinzufügen

	//############ Define input states and events for all levels ############ // nachprüfen ob piltz richtig verwendet
	level(level::off                 ).setInputActions( { ignore(emergencyStop),  ignore(approval) ,  ignore(piltz) });//ignore(laserscanner) });
	level(level::initializing        ).setInputActions( { ignore(emergencyStop),  ignore(approval) ,  ignore(piltz) });//, ignore(laserscanner) });
	level(level::initialized         ).setInputActions( { ignore(emergencyStop),  ignore(approval) ,  ignore(piltz) });//, ignore(laserscanner) });
	level(level::controlStopping     ).setInputActions( { ignore(emergencyStop),  ignore(approval) ,  ignore(piltz) });//, ignore(laserscanner) });
	level(level::powerOn             ).setInputActions( { ignore(emergencyStop),  ignore(approval) ,  check(piltz, false , event::doEmergency) });//, ignore(laserscanner) });
	level(level::homingRad1          ).setInputActions( { ignore(emergencyStop),  ignore(approval) ,  check(piltz, false , event::doEmergency) });//, ignore(laserscanner) });
	level(level::homingRad2          ).setInputActions( { ignore(emergencyStop),  ignore(approval) ,  check(piltz, false , event::doEmergency) });//, ignore(laserscanner) });
	level(level::homingRad3          ).setInputActions( { ignore(emergencyStop),  ignore(approval) ,  check(piltz, false , event::doEmergency) });
	level(level::ready               ).setInputActions( { ignore(emergencyStop),  ignore(approval) ,  check(piltz, false , event::doEmergency) });//, ignore(laserscanner) });
	level(level::driveJoystick       ).setInputActions( { ignore(emergencyStop),  ignore(approval) ,  check(piltz, false , event::doEmergency) });//, ignore(laserscanner) });
	level(level::driveAutonom        ).setInputActions( { ignore(emergencyStop),  ignore(approval) ,  check(piltz, false , event::doEmergency) });//, ignore(laserscanner) });
	level(level::lowEmergency        ).setInputActions( { check(emergencyStop, false, event::resetLowEmergency), ignore(approval) ,  ignore(piltz) });//, ignore(laserscanner) });
	level(level::emergency           ).setInputActions( { check(emergencyStop, false, event::resetEmergency),     ignore(approval) ,  ignore(piltz) });//, ignore(laserscanner) });

	// ############ Define output states and events for all levels ############
	level(level::off                 ).setOutputActions({set(power, false),   set(watchdog, false) });                  
	level(level::initializing        ).setOutputActions({set(power, false),   set(watchdog, false) });                  
	level(level::initialized         ).setOutputActions({set(power, false),   set(watchdog, false) });                  
	level(level::controlStopping     ).setOutputActions({set(power, false),  set(watchdog, false)  });                  
	level(level::powerOn             ).setOutputActions({set(power, false),    toggle(watchdog)    });                  
	                                                                                                                    
	level(level::homingRad1          ).setOutputActions({  set(power, true),  toggle(watchdog) });                      
	level(level::homingRad2          ).setOutputActions({  set(power, true),  toggle(watchdog) });                      
    level(level::homingRad3          ).setOutputActions({  set(power, true),  toggle(watchdog) });                      


	level(level::ready              ).setOutputActions({ set(power, true),   toggle(watchdog)});                                
	level(level::driveJoystick      ).setOutputActions({ set(power, true),   toggle(watchdog)});                                
	level(level::driveAutonom       ).setOutputActions({ set(power, true),   toggle(watchdog)});                                

	level(level::lowEmergency        ).setOutputActions({  set(power, false),     set(watchdog, false),  });                                               
	level(level::emergency           ).setOutputActions({  set(power, false),     toggle(watchdog),      });                                               

	
	
	
	
	// Define and add level functions
	level(level::off).setLevelAction([&](SafetyContext* privateContext) {
		
		
		privateContext->triggerEvent(event::doInit);
	});
	
	
	
	
	level(level::initializing).setLevelAction([&](SafetyContext* privateContext) {
			// Kontrolle für init done
		approvalLED			->set(false);
		emergencyStop_LED	->set(false);
		emergencyLED		->set(false);
		powerOnLED			->set(false);
		
		
		privateContext->triggerEvent(event::InitDone);
	});
	
	
	
	
	
	
	level(level::initialized).setLevelAction([&](SafetyContext* privateContext) {
		
		static int ctr = 0;
		static bool appLED = true;
		
		if ( ctr >= 10) {			
			appLED = !appLED;
		    approvalLED->set(appLED);                                 
			ctr = 0;
			}
			
		ctr++;
		
		
		
		if(approval->get()) {
			
// 			std::cout << "Homed: " << controlSys->axisHomed() << std::endl;
// 			std::cout << "ready: " << controlSys->readyToHoming() << std::endl;
// 			
// 			if(controlSys->axisHomed() == true){
// 				privateContext->triggerEvent(event::doEmergency);
//  			}
// 			
// 			else {
// 				
// 			controlSys->start();
// 
// 			
			privateContext->triggerEvent(event::doPowerOn);
			}

//		}
		
	});

	
	level(level::controlStopping).setLevelAction([&](SafetyContext* privateContext) {
	
// 		controlSys->allAxisStopped();	
// 	
// 		std::cout << "control nicht stopped "  << std::endl;
// 		controlSys->stop();
// 
// 		std::cout << "control stopped "  << std::endl;
		privateContext->triggerEvent(event::controlStoppingDone);
	});
	
	
	level(level::powerOn).setLevelAction([&](SafetyContext* privateContext) {


		std::cout << "In Level powerOn:  "  << std::endl;
		
// 		controlSys->robotControlBlock.homingBlock.swSteer1.switchToInput(0);	// Rad1 steer bekommt torque 
// 		
// 		controlSys->start();
// 		
//		watchdogDevice->reset();

		privateContext->triggerEvent(event::PoweringDone);
	});
	
	level(level::homingRad1).setLevelAction([&](SafetyContext* privateContext) {

		std::cout << "In Level homing1" << std::endl;
		
// 		static int ctr = 0;
// 		
// 		if ( ctr >= 2000) {
//                             
// 			ctr = 0;
// // 			Vector6 volt = controlSys->robotControlBlock.getVoltageOut().getSignal().getValue();
// // 			std::cout << "In Level homing1:  "  <<  "volt(0): "<< volt(0)<< " volt(3) (sollte wert haben): "<< volt(3) << " pwmA3: "<< pwmA3->get() << std::endl;
// // 			std::cout << "In Level homing1:  "  <<  "encoder3 "<< controlSys->enc3.getOut().getSignal().getValue() << " pwmA3: "<< pwmA3->get() << " pwmB3: "<< pwmB3->get()<< "encoder3: "<< controlSys->diffEncPos.getOut().getSignal().getValue()<< std::endl;
// //			std::cout<<  "encoder3 "<< controlSys->enc3.getOut().getSignal().getValue() << " steer1In0: " << controlSys->robotControlBlock.homingBlock.swSteer1.getIn(0).getSignal().getValue() << " steer1In1: " << controlSys->robotControlBlock.homingBlock.swSteer1.getIn(1).getSignal().getValue()<< " steer1Out: " << controlSys->robotControlBlock.homingBlock.swSteer1.getOut().getSignal().getValue()<< std::endl;
// 		
// 			
// 		}
// 		
// 		   ctr++;

     	    privateContext->triggerEvent(event::homingDoneRad1);
			
	});
	
	level(level::homingRad2).setLevelAction([&](SafetyContext* privateContext) {
	// im Sequencer wird in schlaufe gefragt ob homingsensor schon gedrückt ist, soblad gedrückt wird der encoder null gesetzt und das safty in das nächste level
	// Beispiel Homingsequencer von eduro
		std::cout << "In Level homing1" << std::endl;
		
			privateContext->triggerEvent(event::homingDoneRad2);
	});
	
	level(level::homingRad3).setLevelAction([&](SafetyContext* privateContext) {
	// im Sequencer wird in schlaufe gefragt ob homingsensor schon gedrückt ist, soblad gedrückt wird der encoder null gesetzt und das safty in das nächste level
	// Beispiel Homingsequencer von eduro
		
		std::cout << "In Level homing1" << std::endl;
		
			privateContext->triggerEvent(event::homingDoneRad3);
	});
	
	level(level::ready).setLevelAction([&](SafetyContext* privateContext) {
 
			approvalLED			->set(true);
			emergencyStop_LED	->set(true); 
			emergencyLED		->set(true); 
			powerOnLED			->set(true);
 		static int ctr = 0;
// 		static bool appLED;
// 		
		if ( ctr >= 2000) {
                            
			ctr = 0;

			std::cout << "In level read " << std::endl;
			
			}
			
		ctr++;
			

	});
	
	
	level(level::emergency).setLevelAction([&](SafetyContext* privateContext) {
	// im Sequencer wird in schlaufe gefragt ob homingsensor schon gedrückt ist, soblad gedrückt wird der encoder null gesetzt und das safty in das nächste level
	// Beispiel Homingsequencer von eduro
		static int ctr = 0;
		static bool appLED = true;
		
		if ( ctr >= 10) { //TODO anpassen
			appLED = !appLED;
		    emergencyStop_LED->set(appLED);                                 
			ctr = 0;
			}
		
		ctr++;
	});
	
		level(level::lowEmergency).setLevelAction([&](SafetyContext* privateContext) {
	// im Sequencer wird in schlaufe gefragt ob homingsensor schon gedrückt ist, soblad gedrückt wird der encoder null gesetzt und das safty in das nächste level
	// Beispiel Homingsequencer von eduro
		static int ctr = 0;
		static bool appLED = true;
		
		if ( ctr >= 10) { //TODO anpassen
			appLED = !appLED;
		    emergencyStop_LED->set(appLED);                                 
			ctr = 0;
			}
			
		
		ctr++;
	});
	
	// Joystickcontroller wird im Sequencer gestartet mit triggerEvent (mit preconditions sind die bedingungen im sequencer gegeben)
	
}

SafetyWithoutControlSystem::~SafetyWithoutControlSystem() { }






