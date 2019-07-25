#add_library(omnimobot SHARED


find_package(EEROS REQUIRED)
include_directories(${EEROS_INCLUDE_DIR};${EEROS_LIBUCL_INCLUDE_DIR})
link_directories(${EEROS_LIB_DIR};${EEROS_LIBUCL_LINK_DIR})


add_library(omnimobot
	Math.cpp
	Jacobi.cpp
 	# Jacobi_pseudo.cpp
	
# 	control/ControlSystem.cpp
# 	control/ControlSystemWithLaser.cpp
# 	control/ControlSystemWithJacobi.cpp
# 	control/LaserScannerControl.cpp
	control/block/LaserScannerblock.cpp
	control/block/SafetyVeloDesBlock.cpp
	control/block/Jacobian.cpp
	control/block/InvJacobian.cpp
	control/block/MotorModel.cpp
	control/block/TransGtoL.cpp
	control/block/TransLtoG.cpp
	control/block/TransLtoG2.cpp
	control/block/Homingblock.cpp
	control/block/RobotControl.cpp
	control/block/AccTipToPhi.cpp
	control/block/PhiToTip.cpp
	control/block/AccPhiToAccxy.cpp
	control/block/PendulumControl.cpp
	control/block/XBoxInput.cpp
	control/block/PWMblock.cpp
	control/block/VoltageToPWM.cpp
	control/block/KeyboardInput.cpp
	control/block/HallSensorInput.cpp
	control/block/FilterHall.cpp
# 	control/block/TransitionLaserData.cpp
	control/block/TransLtoSapostrophe.cpp
	control/block/TransGtoSapostrophe.cpp
	control/block/TransSapostropheToG.cpp
	control/block/MeasuringDataBlock2.cpp
	control/block/MeasuringDataBlock3.cpp
	JoystickThread.cpp
	Joystick.cpp
	
#  	safety/OmniSafetyProperties.cpp
#  	safety/OmniSafetyPropertiesWithLaser.cpp
#  	safety/SafetyWithControlJacobiProb.cpp

)

target_link_libraries(omnimobot eeros laserscanner ucl ${CMAKE_DL_LIBS}) # Link omnibot library against libeeros

# add_executable(main main.cpp)
# target_link_libraries(main omnimobot)
# 
# add_executable(joystickstart JoystickStart.cpp)
# target_link_libraries(joystickstart omnimobot)
# 
# add_executable(startjoystick StartJoystick.cpp)
# target_link_libraries(startjoystick omnimobot)
