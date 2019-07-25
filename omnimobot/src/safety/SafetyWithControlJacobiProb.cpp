#include <omnimobot/safety/SafetyWithControlJacobiProb.hpp>
#include <omnimobot/control/ControlSystemWithJacobi.hpp>
#include <omnimobot/constants.hpp>


#include <eeros/hal/HAL.hpp>
#include <eeros/safety/inputActions.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <vector>
#include <initializer_list>

#include <unistd.h>
#include <iostream>
//#include <bits/mathcalls.h>

#include <cmath>


using namespace omnimobot;
using namespace eeros::math;

using namespace eeros::logger;

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;

SafetyWithControlJacobiProb::SafetyWithControlJacobiProb(ControlSystemWithJacobi* cs, FlinkWatchdog& motorBoardWatchdog) :  controlSys(cs), watchdog(motorBoardWatchdog),counter(1000)
{ 
	if(controlSys == nullptr) {
		throw -325; // TODO
	}

	
	encOffset.zero();
	
	HAL& hal = HAL::instance();
	
	// ############ Define critical outputs ############
	
	pwmA0 = hal.getRealPeripheralOutput("pwmAaxis0");
	pwmA1 = hal.getRealPeripheralOutput("pwmAaxis1");
	pwmA2 = hal.getRealPeripheralOutput("pwmAaxis2");
	pwmA3 = hal.getRealPeripheralOutput("pwmAaxis3");
	pwmA4 = hal.getRealPeripheralOutput("pwmAaxis4");
	pwmA5 = hal.getRealPeripheralOutput("pwmAaxis5");
	pwmB0 = hal.getRealPeripheralOutput("pwmBaxis0");
	pwmB1 = hal.getRealPeripheralOutput("pwmBaxis1");
	pwmB2 = hal.getRealPeripheralOutput("pwmBaxis2");
	pwmB3 = hal.getRealPeripheralOutput("pwmBaxis3");
	pwmB4 = hal.getRealPeripheralOutput("pwmBaxis4");
	pwmB5 = hal.getRealPeripheralOutput("pwmBaxis5");
	

	powerOnLED =    hal.getLogicPeripheralOutput("powerOnLED");
	emergencyLED =  hal.getLogicPeripheralOutput("emergencyLED");
	
	criticalOutputs = {  powerOnLED, emergencyLED, &watchdog,
						 pwmA0,pwmA1,pwmA2,pwmA3,pwmA4,pwmA5,
						 pwmB0,pwmB1,pwmB2,pwmB3,pwmB4,pwmB5 };

	
	// ############ Define critical inputs ############
	emergencyStop = hal.getLogicPeripheralInput("emergencyStop");
	approval = hal.getLogicPeripheralInput("approval");
	emergencyReset = hal.getLogicPeripheralInput("emergencyReset");
	driveStop = hal.getLogicPeripheralInput("driveStop");
	
	criticalInputs = { emergencyReset, approval, emergencyStop, driveStop};   
	
	
	// ############ Define inputs and Outputs ###########
	
	approvalLED =   hal.getLogicPeripheralOutput("approvalLED");
	stopDriveLED = hal.getLogicPeripheralOutput("stopDriveLED");
	emergencyResetLED =	hal.getLogicPeripheralOutput("emergencyResetLED");
	
	approvalPendulum = hal.getLogicPeripheralInput("approvalPendulum");

	
	homeSensorWheel1   =  hal.getLogicPeripheralInput("Rad1");
	homeSensorWheel2   =  hal.getLogicPeripheralInput("Rad2");
	homeSensorWheel3	 =	hal.getLogicPeripheralInput("Rad3");				
							
	
	
	// ############ Define Levels ############
	levels =
	{
		{ level::off, 			         "Off state", 												 },       
		{ level::initializing,           "software is initializing",               						 },
		{ level::initialized,            "software is initialized / without homing waiting for approval", },
		{ level::lowEmergency,           "low level emergency state",              					     },
		{ level::controlStopping, 	     "Stopping control system", 												 }, 
		{ level::powerOn,                "Power on, controller on, homing starting", 					 }, 
		{ level::homingRad1,             "homing steering 1",                        					 },
		{ level::homingRad2,             "homing steering 2",                      						 },
		{ level::homingRad3,             "homing steering 3",                       						 },
		{ level::emergency,              "emergency state",                          					 },
		{ level::ready,                  "Robot is ready",                           					 },
		{ level::driveJoystick,          "moving with joystick",                     					 },
		{ level::driveAutonom,           "Robot is moving autonomous",             						 },
		{ level::driveJoystickPendulum,  "moving with joystick and pendulum",                    		 },
		{ level::driveAutonomPendulum,   "Robot is moving autonomous and pendulum",             	       }
	};
	
	// ############ Add events to the levels ############
	entryLevel = level::off; // Das erste Level nach dem Start

	level( level::off                     ).addEvent( event::doInit,                   level::initializing,            eeros::safety::kPublicEvent);
	                                                                                                                   
	level( level::initializing            ).addEvent( event::InitDone,                 level::initialized,             eeros::safety::kPublicEvent);
	level( level::initialized             ).addEvent( event::doPowerOn,                level::powerOn,                 eeros::safety::kPublicEvent);
	level( level::initialized             ).addEvent( event::doOff,                    level::off,                     eeros::safety::kPublicEvent);
	                                                                                   
	level( level::controlStopping         ).addEvent( event::controlStoppingDone,      level::off,                     eeros::safety::kPublicEvent);
	                                                                                   
	level( level::powerOn                 ).addEvent( event::PoweringDone,             level::homingRad1,              eeros::safety::kPublicEvent);
	level( level::homingRad1              ).addEvent( event::homingDoneRad1,           level::homingRad2,              eeros::safety::kPublicEvent);
	level( level::homingRad2              ).addEvent( event::homingDoneRad2,           level::homingRad3,              eeros::safety::kPublicEvent);
	level( level::homingRad3              ).addEvent( event::homingDoneRad3,           level::ready,                   eeros::safety::kPublicEvent);
                                                                                                                       
	level( level::ready                   ).addEvent( event::doDriveJoystick,          level::driveJoystick,           eeros::safety::kPublicEvent);
	level( level::ready                   ).addEvent( event::doDriveAutonom,           level::driveAutonom,            eeros::safety::kPublicEvent);
	level( level::ready                   ).addEvent( event::doControlStop,            level::controlStopping,         eeros::safety::kPublicEvent);
	                                                                                                                   
	level( level::driveJoystick           ).addEvent( event::doDriveJoystickPendulum,  level::driveJoystickPendulum,   eeros::safety::kPublicEvent);
	level( level::driveJoystick           ).addEvent( event::stop,                     level::ready,                   eeros::safety::kPublicEvent);
	                                                                                                                   
	level( level::driveAutonom            ).addEvent( event::doDriveAutonomPendulum,   level::driveAutonomPendulum,    eeros::safety::kPublicEvent);
	level( level::driveAutonom            ).addEvent( event::stop,                     level::ready,                   eeros::safety::kPublicEvent);                                                                                                                
	                                                                                                                   
	level( level::driveJoystickPendulum   ).addEvent( event::stop,                     level::ready,                   eeros::safety::kPublicEvent);
	level( level::driveAutonomPendulum    ).addEvent( event::stop,                     level::ready,                   eeros::safety::kPublicEvent);
	                                                                                                                   
	level( level::lowEmergency            ).addEvent( event::doControlStop,            level::controlStopping,         eeros::safety::kPublicEvent);    
 
	level( level::emergency               ).addEvent( event::doControlStop,            level::controlStopping,         eeros::safety::kPublicEvent);
	level( level::emergency               ).addEvent( event::resetEmergency,           level::ready,                   eeros::safety::kPublicEvent);

	
	
	addEventToAllLevelsBetween(level::powerOn, level::homingRad3, event::doEmergency, level::lowEmergency, eeros::safety::kPublicEvent); 
	addEventToLevelAndAbove(level::ready, event::doEmergency, level::emergency, eeros::safety::kPublicEvent); 

	//############ Define input states and events for all levels ############ // nachprüfen ob piltz richtig verwendet
	level(level::off                    ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  ignore(emergencyStop) 							    ,  ignore(driveStop)});
	level(level::initializing           ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  ignore(emergencyStop)							    ,  ignore(driveStop)});
	level(level::initialized            ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  ignore(emergencyStop) 							    ,  ignore(driveStop)});
	level(level::controlStopping        ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  ignore(emergencyStop) 							    ,  ignore(driveStop)});
	level(level::powerOn                ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  ignore(emergencyStop) 							    ,  ignore(driveStop)});
	level(level::homingRad1             ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,   check(emergencyStop, true , event::doEmergency)     ,  ignore(driveStop)});
	level(level::homingRad2             ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,   check(emergencyStop, true , event::doEmergency)     ,  ignore(driveStop)});
	level(level::homingRad3             ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,   check(emergencyStop, true , event::doEmergency)     ,  ignore(driveStop)});
	level(level::ready                  ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,   ignore(emergencyStop)                               , ignore(driveStop)});
	level(level::driveJoystick          ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,   check(emergencyStop, true , event::doEmergency)     ,  ignore(driveStop)});
	level(level::driveAutonom           ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,   check(emergencyStop, true , event::doEmergency)     ,  ignore(driveStop)});
	level(level::driveJoystickPendulum  ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,   check(emergencyStop, true , event::doEmergency)     ,  ignore(driveStop)});
	level(level::driveAutonomPendulum   ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,   check(emergencyStop, true , event::doEmergency)     ,  ignore(driveStop)});
	level(level::lowEmergency           ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  ignore(emergencyStop)                                ,  ignore(driveStop)});
	level(level::emergency              ).setInputActions( { ignore(emergencyReset),     ignore(approval),  ignore(emergencyStop) ,  ignore(driveStop)});

	// ############ Define output states and events for all levels ############
	level(level::off                     ).setOutputActions({set(powerOnLED, false) , set(emergencyLED, false) ,  leave(watchdog)       , set(pwmA0, 0.0), set(pwmA1, 0.0), set(pwmA2, 0.0), set(pwmA3, 0.0), set(pwmA4, 0.0), set(pwmA5, 0.0), set(pwmB0, 0.0), set(pwmB1, 0.0), set(pwmB2, 0.0), set(pwmB3, 0.0), set(pwmB4, 0.0), set(pwmB5, 0.0)});                   /*set(power, false),*/
	level(level::initializing            ).setOutputActions({set(powerOnLED, false) , set(emergencyLED, false) ,  set(watchdog, true)   , set(pwmA0, 0.0), set(pwmA1, 0.0), set(pwmA2, 0.0), set(pwmA3, 0.0), set(pwmA4, 0.0), set(pwmA5, 0.0), set(pwmB0, 0.0), set(pwmB1, 0.0), set(pwmB2, 0.0), set(pwmB3, 0.0), set(pwmB4, 0.0), set(pwmB5, 0.0)});                   /*set(power, false),*/
	level(level::initialized             ).setOutputActions({set(powerOnLED, false) , set(emergencyLED, false) ,  set(watchdog, true)   , set(pwmA0, 0.0), set(pwmA1, 0.0), set(pwmA2, 0.0), set(pwmA3, 0.0), set(pwmA4, 0.0), set(pwmA5, 0.0), set(pwmB0, 0.0), set(pwmB1, 0.0), set(pwmB2, 0.0), set(pwmB3, 0.0), set(pwmB4, 0.0), set(pwmB5, 0.0)});                   /*set(power, false),*/
	level(level::controlStopping         ).setOutputActions({set(powerOnLED, false) , set(emergencyLED, false) ,  set(watchdog, true)   , set(pwmA0, 0.0), set(pwmA1, 0.0), set(pwmA2, 0.0), set(pwmA3, 0.0), set(pwmA4, 0.0), set(pwmA5, 0.0), set(pwmB0, 0.0), set(pwmB1, 0.0), set(pwmB2, 0.0), set(pwmB3, 0.0), set(pwmB4, 0.0), set(pwmB5, 0.0)});                   /*set(power, false),*/
	level(level::powerOn                 ).setOutputActions({set(powerOnLED, false) , set(emergencyLED, false) ,  set(watchdog, true)   , set(pwmA0, 0.0), set(pwmA1, 0.0), set(pwmA2, 0.0), set(pwmA3, 0.0), set(pwmA4, 0.0), set(pwmA5, 0.0), set(pwmB0, 0.0), set(pwmB1, 0.0), set(pwmB2, 0.0), set(pwmB3, 0.0), set(pwmB4, 0.0), set(pwmB5, 0.0)});                   /*set(power, false),*/
									  	  									              //                                                                                                                                                                                                                     /*                  */
	level(level::homingRad1              ).setOutputActions({  set(powerOnLED, true), set(emergencyLED, false) ,  set(watchdog, true)  , leave(pwmA0), leave(pwmA1), leave(pwmA2), leave(pwmA3), leave(pwmA4), leave(pwmA5), leave(pwmB0), leave(pwmB1), leave(pwmB2), leave(pwmB3), leave(pwmB4), leave(pwmB5)});                                                       /*set(power, false),*/
	level(level::homingRad2              ).setOutputActions({  set(powerOnLED, true), set(emergencyLED, false) ,  set(watchdog, true)  , leave(pwmA0), leave(pwmA1), leave(pwmA2), leave(pwmA3), leave(pwmA4), leave(pwmA5), leave(pwmB0), leave(pwmB1), leave(pwmB2), leave(pwmB3), leave(pwmB4), leave(pwmB5)});                                                       /*set(power, false),*/
    level(level::homingRad3              ).setOutputActions({  set(powerOnLED, true), set(emergencyLED, false) ,  set(watchdog, true)  , leave(pwmA0), leave(pwmA1), leave(pwmA2), leave(pwmA3), leave(pwmA4), leave(pwmA5), leave(pwmB0), leave(pwmB1), leave(pwmB2), leave(pwmB3), leave(pwmB4), leave(pwmB5)});                                                          /*set(power, false),*/
                                                                                                  
	level(level::ready                  ).setOutputActions({ set(powerOnLED, true), set(emergencyLED, false) ,   set(watchdog, true)  ,   leave(pwmA0), leave(pwmA1), leave(pwmA2), leave(pwmA3), leave(pwmA4), leave(pwmA5), leave(pwmB0), leave(pwmB1), leave(pwmB2), leave(pwmB3), leave(pwmB4), leave(pwmB5)});    
	level(level::driveJoystick          ).setOutputActions({ set(powerOnLED, true), set(emergencyLED, false) ,   set(watchdog, true)  ,   leave(pwmA0), leave(pwmA1), leave(pwmA2), leave(pwmA3), leave(pwmA4), leave(pwmA5), leave(pwmB0), leave(pwmB1), leave(pwmB2), leave(pwmB3), leave(pwmB4), leave(pwmB5)});    
	level(level::driveAutonom           ).setOutputActions({ set(powerOnLED, true), set(emergencyLED, false) ,   set(watchdog, true)  ,   leave(pwmA0), leave(pwmA1), leave(pwmA2), leave(pwmA3), leave(pwmA4), leave(pwmA5), leave(pwmB0), leave(pwmB1), leave(pwmB2), leave(pwmB3), leave(pwmB4), leave(pwmB5)});                                                                                                                       
	level(level::driveJoystickPendulum  ).setOutputActions({ set(powerOnLED, true), set(emergencyLED, false) ,   set(watchdog, true)  ,   leave(pwmA0), leave(pwmA1), leave(pwmA2), leave(pwmA3), leave(pwmA4), leave(pwmA5), leave(pwmB0), leave(pwmB1), leave(pwmB2), leave(pwmB3), leave(pwmB4), leave(pwmB5)});    
	level(level::driveAutonomPendulum   ).setOutputActions({ set(powerOnLED, true), set(emergencyLED, false) ,   set(watchdog, true)  ,   leave(pwmA0), leave(pwmA1), leave(pwmA2), leave(pwmA3), leave(pwmA4), leave(pwmA5), leave(pwmB0), leave(pwmB1), leave(pwmB2), leave(pwmB3), leave(pwmB4), leave(pwmB5)});                                                                                                                       
                                                                                        

	level(level::lowEmergency        ).setOutputActions({  set(powerOnLED, false),  set(emergencyLED, true) ,   leave(watchdog) ,   leave(pwmA0), leave(pwmA1), leave(pwmA2), leave(pwmA3), leave(pwmA4), leave(pwmA5), leave(pwmB0), leave(pwmB1), leave(pwmB2), leave(pwmB3), leave(pwmB4), leave(pwmB5)});                                                     /*toggle(power),*/
	level(level::emergency           ).setOutputActions({  set(powerOnLED, false),  set(emergencyLED, true) ,   leave(watchdog) ,   leave(pwmA0), leave(pwmA1), leave(pwmA2), leave(pwmA3), leave(pwmA4), leave(pwmA5), leave(pwmB0), leave(pwmB1), leave(pwmB2), leave(pwmB3), leave(pwmB4), leave(pwmB5)});                                                    /*toggle(power),*/

	
	
	
	
	// Define and add level functions
	level(level::off).setLevelAction([&](SafetyContext* privateContext) {
		
		static bool first = true; 
		if(first == true) {
			privateContext->triggerEvent(event::doInit);
			first = false;
		}
	});
	
	
	
	level(level::initializing).setLevelAction([&](SafetyContext* privateContext) {
	// Kontrolle für init done
		approvalLED			->set(false);
		emergencyResetLED	->set(false);
		stopDriveLED		->set(false);
		powerOnLED			->set(false);
		
		watchdog.reset();
		
		privateContext->triggerEvent(event::InitDone);
	});
	
	
	
	level(level::initialized).setLevelAction([&](SafetyContext* privateContext) {
		// wait of the approval
		if(emergencyStop->get()) {     
			

			if(controlSys->isaxisHomed() == true){
				privateContext->triggerEvent(event::doEmergency);
			}
			else {
			
				privateContext->triggerEvent(event::doPowerOn);
											 
			}
		}
	});

	
	
	level(level::controlStopping).setLevelAction([&](SafetyContext* privateContext) {
		// Stop Axis and control
		controlSys->allAxisStopped();	
		usleep(2000);
		controlSys->stop();

		std::cout << "control stopped "  << std::endl;
		privateContext->triggerEvent(event::controlStoppingDone);
	});
	
	
	
	level(level::powerOn).setLevelAction([&](SafetyContext* privateContext) {
		// Homing get init, Wheel 1 will homing and watchdog is reset
		controlSys->initHoming();
		controlSys->setWheelToHoming(steer1);
	
		controlSys->start();

		privateContext->triggerEvent(event::PoweringDone);
	});
	
	
	
	level(level::homingRad1).setLevelAction([&](SafetyContext* privateContext) {
		static bool firstDedected = true; 
		if(!firstDedected){
			counter.count();
		}
		
		// if Homingsensor dedected set the offset and change the level to homingRad2
		if(homeSensorWheel1->get()){ 
			 
		
			 if(firstDedected) {
				firstDedected = false;
			 }
			 
			 if(counter.isCountEnd()) {
				counter.reset();
				 // close output for steer1
				controlSys->homingBlock.swSteer1.switchToInput(1); 
				// Offset Wheel1
				encOffset(steer1) = -controlSys->integratorQpointToQ.getOut().getSignal().getValue()(steer1) + offsetSteer1;//+ 1.776;//offSteer1;//1.776;//+ offsetSteer1; 
				// Offset Wheel2
				encOffset(steer2) = -controlSys->integratorQpointToQ.getOut().getSignal().getValue()(steer2);
				controlSys->offsetEnc.setValue(encOffset);
				
				usleep(50);
				controlSys->setWheelToHoming(steer2);
				privateContext->triggerEvent(event::homingDoneRad1);
			 }
		 }
	});
	
	
	
	level(level::homingRad2).setLevelAction([&](SafetyContext* privateContext) {
		static bool firstDedected = true;
	
		if(!firstDedected){
			counter.count();
		}
				
		 if(homeSensorWheel2->get()){
			 
			 if(firstDedected) {
				firstDedected = false;
			 }
			 
			 if(counter.isCountEnd()) {
				counter.reset(); 
				
				controlSys->homingBlock.swSteer2.switchToInput(1); // close output for steer2
				encOffset(steer2) = -controlSys->integratorQpointToQ.getOut().getSignal().getValue()(steer2) + offsetSteer2; // Offset Rad2
				encOffset(steer3) = -controlSys->integratorQpointToQ.getOut().getSignal().getValue()(steer3);
				controlSys->offsetEnc.setValue(encOffset);
				usleep(50);
				controlSys->setWheelToHoming(steer3);
				privateContext->triggerEvent(event::homingDoneRad2);
			 }
		 }
	});
	
	
	
	level(level::homingRad3).setLevelAction([&](SafetyContext* privateContext) {
		static bool firstDedected = true;
		
		if(!firstDedected){
			counter.count();
		}	
		
		 if(homeSensorWheel3->get()){
			 
			 if(firstDedected) {
				firstDedected = false;
			 }
			 
			 if(counter.isCountEnd())  {
				counter.reset(); 
				
				controlSys->homingBlock.swSteer3.switchToInput(1); // close output for steer3
				encOffset(steer3) = -controlSys->integratorQpointToQ.getOut().getSignal().getValue()(steer3)  +offsetSteer3; 
				controlSys->offsetEnc.setValue(encOffset);
				usleep(50);
				
				controlSys->setHomingToTrue();
				
				privateContext->triggerEvent(event::homingDoneRad3);
			 }
		 }
	});
	
	
	
	level(level::ready).setLevelAction([&](SafetyContext* privateContext) {
		if (!emergencyStop->get()){
			
			static int ctr = 0;
			static bool resetLED = true;
			
			// LED toggeln
			if ( ctr >= 500) {
				
				resetLED = !resetLED;
				emergencyLED->set(resetLED);                                 

				std::cout << " emergencyStop loesen und Reset druecken " <<std::endl;

				ctr = 0;
			}
			ctr++;
		}
		else{
			// LED toggeln
			static int ctr1 = 0;
			static bool appLED = true;
			
			if ( ctr1 >= 500) {			
				appLED = !appLED;
				approvalLED->set(appLED);                                 
				ctr1 = 0;
			}
				
			ctr1++;
		}

		// wait of the approval for Joystick drive
		if(approval->get() && emergencyStop->get()) {
			approvalLED->set(false);
			controlSys->initReadyToDrive();
			
			if(!controlSys->isaxisHomed() || !controlSys->isreadyToDrive()){
				privateContext->triggerEvent(event::doEmergency);
			}
			else {
				
				controlSys->swHomingOn.switchToInput(0);
				
				privateContext->triggerEvent(event::doDriveJoystick);
				usleep(50);
			}
		}
	});
	
	
	
	level(level::driveJoystick).setLevelAction([&](SafetyContext* privateContext) {

		static int ctr = 0;                  
		static bool LEDTmp = true;           

		if ( ctr >= 500) { //TODO anpassen  
			LEDTmp = !LEDTmp;                
		    stopDriveLED->set(LEDTmp);       
			
// 			auto v = controlSys->keyboadIn.getOut().getSignal().getValue();
// 			std::cout << v[0] << "   " << v[1] << "   " << v[3] << "   integratorin:  " << controlSys->integratorOdoPointToOdo.getIn().getSignal().getValue()(0) << "   integratorOut:  " << controlSys->integratorOdoPointToOdo.getOut().getSignal().getValue()(0) << "   ltoG in vx:  " << controlSys->localToGlobal.getInVL().getSignal().getValue()(0) <<"  ltoG in phi:    "<< controlSys->localToGlobal.getInPhiGL().getSignal().getValue()<<"  ltoG out x:    "<< controlSys->localToGlobal.getOutVG().getSignal().getValue()(0)<< std::endl;
// 			std::cout << "  jacobi out x:  "<< controlSys->jacobi.getOutVL().getSignal().getValue()(0)<< "  jacobi out y:  "<< controlSys->jacobi.getOutVL().getSignal().getValue()(1) << "  jacobi out phid:  "<< controlSys->jacobi.getOutVL().getSignal().getValue()(2)  <<  "   integratorin tmp:  " << controlSys->integratorOdoPointToOdo.getIn().getSignal().getTimestamp() << "   integrator in value phid:  " << controlSys->integratorOdoPointToOdo.getIn().getSignal().getValue()(2) << "   integratorOutvalue x:  " << controlSys->integratorOdoPointToOdo.getOut().getSignal().getValue()(0) <<"  jacobi out timestamp:    "<< controlSys->jacobi.getOutVL().getSignal().getTimestamp()<< std::endl;
// 			std::cout << v[0] << "   " << v[1] << "   " << v[3] << "   integratorOut:  " << controlSys->integratorOdoPointToOdo.getOut().getSignal().getValue()(0) << std::endl;	
			ctr = 0;
		}
		
		ctr++;

		if(controlSys->joystick.isStop()) {
			stopDriveLED->set(false); 
			auto &counter = controlSys->timedomain.counter;
			std::cout << "drive esc"<< "    min run: " << counter.run.min << " max run: " << counter.run.max << "    period max: " << counter.period.max << "    period min: " << counter.period.min << "    period mean: " << counter.period.mean <<std::endl;
			counter.run.reset();
			counter.jitter.reset();		
			counter.period.reset();
			
			privateContext->triggerEvent(event::doEmergency);
		}
		
		if(driveStop->get()) {
			stopDriveLED->set(false); 
			auto &counter = controlSys->timedomain.counter;
			std::cout << "drive stop"<< "    min run: " << counter.run.min << " max run: " << counter.run.max << "    period max: " << counter.period.max << "    period min: " << counter.period.min << "    period mean: " << counter.period.mean <<std::endl;
			counter.run.reset();
			counter.jitter.reset();		
			counter.period.reset();
			controlSys->allAxisStopped();
			usleep(2000);
			privateContext->triggerEvent(event::stop);
		}
		
	});
	
	
	
	level(level::driveJoystickPendulum).setLevelAction([&](SafetyContext* privateContext) {
	// not possible in this safety system
		static int ctr = 0;                  
		static bool LEDTmp = true;           
		                                     
		if ( ctr >= 500) { //TODO anpassen  
			LEDTmp = !LEDTmp;                
		    stopDriveLED->set(LEDTmp);       
			
			ctr = 0;
			}
		
		ctr++;

		if(controlSys->joystick.isStop()) {
			auto &counter = controlSys->timedomain.counter;
			std::cout << "pendlum esc"<< "    min run: " << counter.run.min << " max run: " << counter.run.max << "    period max: " << counter.period.max << "    period min: " << counter.period.min << "    period mean: " << counter.period.mean <<std::endl;
			counter.run.reset();
			counter.jitter.reset();		
			counter.period.reset();
			controlSys->swPendulumOn.switchToInput(0);
			stopDriveLED->set(false); 
			privateContext->triggerEvent(event::doEmergency);
		}
		
	});
	
	
	
	level(level::emergency).setLevelAction([&](SafetyContext* privateContext) {
		// is homed
		if (!controlSys->isaxisHomed()){
			emergencyResetLED->set(false);
			privateContext->triggerEvent(event::doControlStop);
		}
		
		if (emergencyReset->get()){
			emergencyResetLED->set(false);
			controlSys->allAxisStopped();
			watchdog.reset();
			privateContext->triggerEvent(event::resetEmergency);
		}
		// LED toggeln
		static int ctr = 0;
		static bool emLED = true;
		
		if ( ctr >= 500) { 
			emLED = !emLED;
		    emergencyResetLED->set(emLED);                                 
			ctr = 0;
			}
		
		ctr++;
	});
	
	
	
	level(level::lowEmergency).setLevelAction([&](SafetyContext* privateContext) {
			
		privateContext->triggerEvent(event::doControlStop);		
	});
}

SafetyWithControlJacobiProb::~SafetyWithControlJacobiProb() { }






