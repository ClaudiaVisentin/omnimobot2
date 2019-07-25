#include <omnimobot/safety/OmniSafetyPropertiesWithLaser.hpp>
#include <omnimobot/control/ControlSystemWithLaser.hpp>
#include <omnimobot/constants.hpp>

#include <eeros/safety/inputActions.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <vector>
#include <initializer_list>

#include <unistd.h>
#include <iostream>
#include <chrono>

using clk = std::chrono::high_resolution_clock;
using duration = std::chrono::duration<double>;

using namespace omnimobot;
using namespace eeros;
using namespace eeros::math;
using namespace eeros::hal;
using namespace eeros::safety;

OmniSafetyPropertiesWithLaser::OmniSafetyPropertiesWithLaser(ControlSystemWithLaser* cs, FlinkWatchdog& motorBoardWatchdog) :  controlSys(cs), watchdog(motorBoardWatchdog),counter(1000)
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
	
	criticalInputs = { emergencyReset, approval, emergencyStop};   
	
	
	// ############ Define inputs and Outputs ###########
	
	approvalLED =   hal.getLogicPeripheralOutput("approvalLED");
	stopDriveLED = hal.getLogicPeripheralOutput("stopDriveLED");
	emergencyResetLED =	hal.getLogicPeripheralOutput("emergencyResetLED");
	
	approvalPendulum = hal.getLogicPeripheralInput("approvalPendulum");
	driveStop = hal.getLogicPeripheralInput("driveStop");
	
	homeSensorWheel1   =  hal.getLogicPeripheralInput("Rad1");
	homeSensorWheel2   =  hal.getLogicPeripheralInput("Rad2");
	homeSensorWheel3	 =	hal.getLogicPeripheralInput("Rad3");				
							
	
	// ############ Define Levels ############
	
	levels =
	{
		{ level::off, 			         "Off state", 												      },       
		{ level::initializing,           "software is initializing",               						  },
		{ level::initialized,            "software is initialized / without homing waiting for approval", },
		{ level::lowEmergency,           "low level emergency state",              					      },
		{ level::controlStopping, 	     "Stopping control system", 									  }, 
		{ level::powerOn,                "Power on, controller on, homing starting", 					  }, 
		{ level::homingRad1,             "homing steering 1",                        					  },
		{ level::homingRad2,             "homing steering 2",                      						  },
		{ level::homingRad3,             "homing steering 3",                       					  },
		{ level::safetyStop,             "safety stop (all drives set 0)",            					  },
		{ level::emergency,              "emergency state",                          					  },
		{ level::ready,                  "Robot is ready",                           					  },
		{ level::driveJoystick,          "moving with joystick",                     					  },
		{ level::driveAutonom,           "Robot is moving autonomous",             						  },
		{ level::driveJoystickPendulum,  "moving with joystick and pendulum",                    		  },
		{ level::driveAutonomPendulum,   "Robot is moving autonomous and pendulum",             	      }
	};
	
	
	// ############ Add events to the levels ############
	
	entryLevel = level::off; // the first level

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
	level( level::safetyStop			  ).addEvent( event::setEmergency,             level::emergency,               eeros::safety::kPublicEvent);
	level( level::emergency               ).addEvent( event::doControlStop,            level::controlStopping,         eeros::safety::kPublicEvent);
	level( level::emergency               ).addEvent( event::resetEmergency,           level::ready,                   eeros::safety::kPublicEvent);

	addEventToAllLevelsBetween(level::powerOn, level::homingRad3, event::doEmergency, level::lowEmergency, eeros::safety::kPublicEvent); // Add the event doEmergency to a level area
	addEventToLevelAndAbove(level::ready, event::doEmergency, level::safetyStop, eeros::safety::kPublicEvent); 							 // Add the event doEmergency from ready

	level(level::off                    ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  ignore(emergencyStop) 						   });
	level(level::initializing           ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  ignore(emergencyStop)							   });
	level(level::initialized            ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  ignore(emergencyStop) 						   });
	level(level::controlStopping        ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  ignore(emergencyStop) 						   });
	level(level::powerOn                ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  ignore(emergencyStop) 						   });
	level(level::homingRad1             ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  check(emergencyStop, true , event::doEmergency)  });
	level(level::homingRad2             ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  check(emergencyStop, true , event::doEmergency)  });
	level(level::homingRad3             ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  check(emergencyStop, true , event::doEmergency)  });
	level(level::ready                  ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  ignore(emergencyStop) 						   });
	level(level::driveJoystick          ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  check(emergencyStop, true , event::doEmergency)  });
	level(level::driveAutonom           ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  check(emergencyStop, true , event::doEmergency)  });
	level(level::driveJoystickPendulum  ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  check(emergencyStop, true , event::doEmergency)  });
	level(level::driveAutonomPendulum   ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  check(emergencyStop, true , event::doEmergency)  });
	level(level::lowEmergency           ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  ignore(emergencyStop)                            });
	level(level::safetyStop             ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  ignore(emergencyStop)                            });
	level(level::emergency              ).setInputActions( { ignore(emergencyReset),  ignore(approval) ,  ignore(emergencyStop)                            });

	
	// ############ Define output states and events for all levels ############
	
	level(level::off                     ).setOutputActions({set(powerOnLED, false) , set(emergencyLED, false), leave(watchdog)    , set(pwmA0, 0.0), set(pwmA1, 0.0), set(pwmA2, 0.0), set(pwmA3, 0.0), set(pwmA4, 0.0), set(pwmA5, 0.0), set(pwmB0, 0.0), set(pwmB1, 0.0), set(pwmB2, 0.0), set(pwmB3, 0.0), set(pwmB4, 0.0), set(pwmB5, 0.0)});                
	level(level::initializing            ).setOutputActions({set(powerOnLED, false) , set(emergencyLED, false), set(watchdog, true), set(pwmA0, 0.0), set(pwmA1, 0.0), set(pwmA2, 0.0), set(pwmA3, 0.0), set(pwmA4, 0.0), set(pwmA5, 0.0), set(pwmB0, 0.0), set(pwmB1, 0.0), set(pwmB2, 0.0), set(pwmB3, 0.0), set(pwmB4, 0.0), set(pwmB5, 0.0)});                
	level(level::initialized             ).setOutputActions({set(powerOnLED, false) , set(emergencyLED, true),  set(watchdog, true), set(pwmA0, 0.0), set(pwmA1, 0.0), set(pwmA2, 0.0), set(pwmA3, 0.0), set(pwmA4, 0.0), set(pwmA5, 0.0), set(pwmB0, 0.0), set(pwmB1, 0.0), set(pwmB2, 0.0), set(pwmB3, 0.0), set(pwmB4, 0.0), set(pwmB5, 0.0)});                
	level(level::controlStopping         ).setOutputActions({set(powerOnLED, false) , set(emergencyLED, false), set(watchdog, true), set(pwmA0, 0.0), set(pwmA1, 0.0), set(pwmA2, 0.0), set(pwmA3, 0.0), set(pwmA4, 0.0), set(pwmA5, 0.0), set(pwmB0, 0.0), set(pwmB1, 0.0), set(pwmB2, 0.0), set(pwmB3, 0.0), set(pwmB4, 0.0), set(pwmB5, 0.0)});                
	level(level::powerOn                 ).setOutputActions({set(powerOnLED, false) , set(emergencyLED, false), set(watchdog, true), set(pwmA0, 0.0), set(pwmA1, 0.0), set(pwmA2, 0.0), set(pwmA3, 0.0), set(pwmA4, 0.0), set(pwmA5, 0.0), set(pwmB0, 0.0), set(pwmB1, 0.0), set(pwmB2, 0.0), set(pwmB3, 0.0), set(pwmB4, 0.0), set(pwmB5, 0.0)});                
									  	  									                                                                                                                                                                                                                              
	level(level::homingRad1              ).setOutputActions({set(powerOnLED, true)  , set(emergencyLED, false), set(watchdog, true), leave(pwmA0),    leave(pwmA1),    leave(pwmA2),    leave(pwmA3),    leave(pwmA4),    leave(pwmA5),    leave(pwmB0),    leave(pwmB1),    leave(pwmB2),    leave(pwmB3),    leave(pwmB4),    leave(pwmB5)});                               
	level(level::homingRad2              ).setOutputActions({set(powerOnLED, true)  , set(emergencyLED, false), set(watchdog, true), leave(pwmA0),    leave(pwmA1),    leave(pwmA2),    leave(pwmA3),    leave(pwmA4),    leave(pwmA5),    leave(pwmB0),    leave(pwmB1),    leave(pwmB2),    leave(pwmB3),    leave(pwmB4),    leave(pwmB5)});                               
    level(level::homingRad3              ).setOutputActions({set(powerOnLED, true)  , set(emergencyLED, false), set(watchdog, true), leave(pwmA0),    leave(pwmA1),    leave(pwmA2),    leave(pwmA3),    leave(pwmA4),    leave(pwmA5),    leave(pwmB0),    leave(pwmB1),    leave(pwmB2),    leave(pwmB3),    leave(pwmB4),    leave(pwmB5)});                               
                                                                                                                                                                                                                                                                                                                                
	level(level::ready                   ).setOutputActions({set(powerOnLED, true)  , leave(emergencyLED),      set(watchdog, true), leave(pwmA0),    leave(pwmA1),    leave(pwmA2),    leave(pwmA3),    leave(pwmA4),    leave(pwmA5),    leave(pwmB0),    leave(pwmB1),    leave(pwmB2),    leave(pwmB3),    leave(pwmB4),    leave(pwmB5)});    
	level(level::driveJoystick           ).setOutputActions({set(powerOnLED, true)  , set(emergencyLED, false), set(watchdog, true), leave(pwmA0),    leave(pwmA1),    leave(pwmA2),    leave(pwmA3),    leave(pwmA4),    leave(pwmA5),    leave(pwmB0),    leave(pwmB1),    leave(pwmB2),    leave(pwmB3),    leave(pwmB4),    leave(pwmB5)});    
	level(level::driveAutonom            ).setOutputActions({set(powerOnLED, true)  , set(emergencyLED, false), set(watchdog, true), leave(pwmA0),    leave(pwmA1),    leave(pwmA2),    leave(pwmA3),    leave(pwmA4),    leave(pwmA5),    leave(pwmB0),    leave(pwmB1),    leave(pwmB2),    leave(pwmB3),    leave(pwmB4),    leave(pwmB5)});                                                                                                                       
	level(level::driveJoystickPendulum   ).setOutputActions({set(powerOnLED, true)  , set(emergencyLED, false), set(watchdog, true), leave(pwmA0),    leave(pwmA1),    leave(pwmA2),    leave(pwmA3),    leave(pwmA4),    leave(pwmA5),    leave(pwmB0),    leave(pwmB1),    leave(pwmB2),    leave(pwmB3),    leave(pwmB4),    leave(pwmB5)});    
	level(level::driveAutonomPendulum    ).setOutputActions({set(powerOnLED, true)  , set(emergencyLED, false), set(watchdog, true), leave(pwmA0),    leave(pwmA1),    leave(pwmA2),    leave(pwmA3),    leave(pwmA4),    leave(pwmA5),    leave(pwmB0),    leave(pwmB1),    leave(pwmB2),    leave(pwmB3),    leave(pwmB4),    leave(pwmB5)});                                                                                                                       
                                                                                        
	level(level::lowEmergency            ).setOutputActions({set(powerOnLED, false) , set(emergencyLED, true) , leave(watchdog) ,    set(pwmA0, 0.0), set(pwmA1, 0.0), set(pwmA2, 0.0), set(pwmA3, 0.0), set(pwmA4, 0.0), set(pwmA5, 0.0), set(pwmB0, 0.0), set(pwmB1, 0.0), set(pwmB2, 0.0), set(pwmB3, 0.0), set(pwmB4, 0.0), set(pwmB5, 0.0)});                                                  
	level(level::safetyStop              ).setOutputActions({set(powerOnLED, true)  , set(emergencyLED, true) , set(watchdog, true), leave(pwmA0),    leave(pwmA1),    leave(pwmA2),    leave(pwmA3),    leave(pwmA4),    leave(pwmA5),    leave(pwmB0),    leave(pwmB1),    leave(pwmB2),    leave(pwmB3),    leave(pwmB4),    leave(pwmB5)});                                                  
	level(level::emergency               ).setOutputActions({set(powerOnLED, false) , set(emergencyLED, true) , leave(watchdog) ,    set(pwmA0, 0.0), set(pwmA1, 0.0), set(pwmA2, 0.0), set(pwmA3, 0.0), set(pwmA4, 0.0), set(pwmA5, 0.0), set(pwmB0, 0.0), set(pwmB1, 0.0), set(pwmB2, 0.0), set(pwmB3, 0.0), set(pwmB4, 0.0), set(pwmB5, 0.0)});                                                


	// ############ Define and add level functions ############
	
	level(level::off).setLevelAction([&](SafetyContext* privateContext) {
		
		// light of
		static bool first = true; 
		
		approvalLED			->set(false);
		emergencyResetLED	->set(false);
		stopDriveLED		->set(false);
		
		
		if(first == true) {
			first = false;
			privateContext->triggerEvent(event::doInit);
		}
	});
	
	
	
	level(level::initializing).setLevelAction([&](SafetyContext* privateContext) {
		// watchdog is reset
		watchdog.reset();
		
		privateContext->triggerEvent(event::InitDone);
	});
	
	
	
	level(level::initialized).setLevelAction([&](SafetyContext* privateContext) {
		// wait of the approval
		if(emergencyStop->get()) {         // reset the safety circuit 

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
		// Homing get init, Wheel 1 will homing 
		controlSys->initHoming();
		controlSys->setWheelToHoming(steer1);
	
		controlSys->start();	
		
		// plot run and period time
		auto &counter = controlSys->timedomain.counter;
		std::cout << "power on " << "    min run: " << counter.run.min << " max run: " << counter.run.max << "    period max: " << counter.period.max << "    period min: " << counter.period.min << "    period mean: " << counter.period.mean <<std::endl;
		counter.run.reset();
		counter.jitter.reset();
		counter.period.reset();

		privateContext->triggerEvent(event::PoweringDone);
	});
	
	
	
	level(level::homingRad1).setLevelAction([&](SafetyContext* privateContext) {
		// homing steer1
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
				controlSys->robotControlBlock.homingBlock.swSteer1.switchToInput(1); 
				// Offset Wheel1
				encOffset(steer1) = -controlSys->integratorQpoint.getOut().getSignal().getValue()(steer1) + offsetSteer1;
				// Offset Wheel2
				encOffset(steer2) = -controlSys->integratorQpoint.getOut().getSignal().getValue()(steer2);
				controlSys->offsetEnc.setValue(encOffset);
		
				usleep(50);
				controlSys->setWheelToHoming(steer2);
				privateContext->triggerEvent(event::homingDoneRad1);
			 }
		 }
	});
	
	
	
	level(level::homingRad2).setLevelAction([&](SafetyContext* privateContext) {
		// homing steer2
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
							
				controlSys->robotControlBlock.homingBlock.swSteer2.switchToInput(1); 
				encOffset(steer2) = -controlSys->integratorQpoint.getOut().getSignal().getValue()(steer2) + offsetSteer2; 
				encOffset(steer3) = -controlSys->integratorQpoint.getOut().getSignal().getValue()(steer3);
				controlSys->offsetEnc.setValue(encOffset);
				
				usleep(50);
				controlSys->setWheelToHoming(steer3);
				privateContext->triggerEvent(event::homingDoneRad2);
			 }
		 }
	});
	
	
	
	level(level::homingRad3).setLevelAction([&](SafetyContext* privateContext) {
		// homing steer3
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
				
				controlSys->robotControlBlock.homingBlock.swSteer3.switchToInput(1); 
				encOffset(steer3) = -controlSys->integratorQpoint.getOut().getSignal().getValue()(steer3)  +offsetSteer3;
				controlSys->offsetEnc.setValue(encOffset);
	
				usleep(50);
				
				controlSys->setHomingToTrue();
				controlSys->robotControlBlock.integrator.disable();
				
				auto &counter = controlSys->timedomain.counter;
				counter.run.reset();
				counter.jitter.reset();		
				counter.period.reset();
				
				privateContext->triggerEvent(event::homingDoneRad3);
			 }
		 }
	});
	
	
	
	level(level::ready).setLevelAction([&](SafetyContext* privateContext) {
		// if emergencyStop
		if (!emergencyStop->get()){
			static int ctr = 0;
			static bool resetLED = true;
			
			// resetLED toggeln
			if ( ctr >= 500) {
				resetLED = !resetLED;
				emergencyLED->set(resetLED);                                 

				std::cout << " emergencyStop loesen und Reset druecken " <<std::endl;

				ctr = 0;
			}
			ctr++;
		}
		else{
			// approvalLED toggeln
			static int ctr1 = 0;
			static bool appLED = true;
			
			if ( ctr1 >= 500) {			
				appLED = !appLED;
				approvalLED->set(appLED);  
				ctr1 = 0;
				
// 				std::cout << " out LtoS: " <<controlSys->transLtoS.getOutVSapostrophe().getSignal().getValue()(0)<<"   out lasertrans: "<<controlSys->transLaserData.getOutCollDataSapostrophe().getSignal().getValue()(0,3)<<std::endl;
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
				controlSys->robotControlBlock.swHomingOn.switchToInput(0);
				
				// plot run and period time
				auto &counter = controlSys->timedomain.counter;
				std::cout << "ready to drive"<< "    min run: " << counter.run.min << " max run: " << counter.run.max << "    period max: " << counter.period.max << "    period min: " << counter.period.min << "    period mean: " << counter.period.mean <<std::endl;
				counter.run.reset();
				counter.jitter.reset();		
				counter.period.reset();
				
				privateContext->triggerEvent(event::doDriveJoystick);
			}
		}
	});
	
	
	
	level(level::driveJoystick).setLevelAction([&](SafetyContext* privateContext) {
		// drive
		static int ctr = 0;                  
		static bool LEDTmp = true;           
		       
		// stopDriveLED toggeln
		if ( ctr >= 500) { 
			LEDTmp = !LEDTmp;                
		    stopDriveLED->set(LEDTmp);       
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
		
		// wait of the approvalPendulum for Joystick drive (T4)
		if(approvalPendulum->get() && emergencyStop->get() && controlSys->hallSensorInput.isStickConnected()) {
			stopDriveLED->set(false); 
			auto &counter = controlSys->timedomain.counter;
			std::cout << "drive vor set pendlum"<< "    min run: " << counter.run.min << " max run: " << counter.run.max << "    period max: " << counter.period.max << "    period min: " << counter.period.min << "    period mean: " << counter.period.mean <<std::endl;
			counter.run.reset();
			counter.jitter.reset();		
			counter.period.reset();
			
			controlSys->initReadyToPendulum();
			
			std::cout << "drive nach set pendlum"<< "    min run: " << counter.run.min << " max run: " << counter.run.max << "    period max: " << counter.period.max << "    period min: " << counter.period.min << "    period mean: " << counter.period.mean <<std::endl;
			counter.run.reset();
			counter.jitter.reset();		
			counter.period.reset();
			
			privateContext->triggerEvent(event::doDriveJoystickPendulum);			
		}
	});

	
	
	level(level::driveJoystickPendulum).setLevelAction([&](SafetyContext* privateContext) {
		static int ctr = 0;   
// 		static int ctr1 = 0;
		static bool LEDTmp = true;           
		
		// stopDriveLED toggeln
		if ( ctr >= 500) {   
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
			controlSys->robotControlBlock.swSafetystopOn.switchToInput(1);
			stopDriveLED->set(false); 
			privateContext->triggerEvent(event::doEmergency);
		}
		
		if(!controlSys->hallSensorInput.isStickConnected()) {
			auto &counter = controlSys->timedomain.counter;
			std::cout << "pendlum stick"<< "    min run: " << counter.run.min << " max run: " << counter.run.max << "    period max: " << counter.period.max << "    period min: " << counter.period.min << "    period mean: " << counter.period.mean <<std::endl;
			counter.run.reset();
			counter.jitter.reset();		
			counter.period.reset();
			
			controlSys->robotControlBlock.swSafetystopOn.switchToInput(1);
			
			if (controlSys->jacobi.getOutVL().getSignal().getValue()(0) <= 0.0005 && controlSys->jacobi.getOutVL().getSignal().getValue()(1) <= 0.0005 && controlSys->jacobi.getOutVL().getSignal().getValue()(2) <= 0.0005) {
				usleep(1000);
				controlSys->allAxisStopped();
				std::cout << "   lost stick  " <<controlSys->hallSensorInput.isStickConnected()<<  std::endl;
				stopDriveLED->set(false); 
				usleep(1000);
				privateContext->triggerEvent(event::stop);
			}
			else {
				usleep(2000);
			}
		}
	});
	
	
	
	level(level::lowEmergency).setLevelAction([&](SafetyContext* privateContext) {
			
		privateContext->triggerEvent(event::doControlStop);		
	});
	
	
	
	level(level::safetyStop).setLevelAction([&](SafetyContext* privateContext) {
		static int ctr = 0;
		
		controlSys->robotControlBlock.swSafetystopOn.switchToInput(1);
		
		if (controlSys->robotControlBlock.swSafetystopOn.getOut().getSignal().getValue()(0) == 0.0 && controlSys->robotControlBlock.swSafetystopOn.getOut().getSignal().getValue()(3) == 0.0 || ctr >= 10) {
			ctr = 0;
			privateContext->triggerEvent(event::setEmergency);
		}
		ctr++;
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
		
		// emergencyResetLED toggeln
		static int ctr = 0;
		static bool emLED = true;
		
		if ( ctr >= 500) { 
			emLED = !emLED;
		    emergencyResetLED->set(emLED);                                 
			ctr = 0;
			}
		ctr++;
	});
	
	
	

}

OmniSafetyPropertiesWithLaser::~OmniSafetyPropertiesWithLaser() { }






