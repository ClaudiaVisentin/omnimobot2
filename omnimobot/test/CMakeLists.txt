# Unit tests for the omnimobot project

# Build and link test executables

add_executable(i-test test-I.cpp)
target_link_libraries(i-test omnimobot eeros)

add_executable(Inv-jacobian-test test-InvJacobian.cc)
target_link_libraries(Inv-jacobian-test omnimobot eeros)

add_executable(test-runTimeGainMatrix test-RunTimeGainMatrix.cpp)
target_link_libraries(test-runTimeGainMatrix eeros)

add_executable(test-runTimeGainVector test-RunTimeGainVector.cpp)
target_link_libraries(test-runTimeGainVector eeros)

add_executable(test-homeSensor test-Input.cpp)
target_link_libraries(test-homeSensor  omnimobot eeros)

add_executable(test-joystick test-JoystickThread.cpp)
target_link_libraries(test-joystick  omnimobot eeros)

add_executable(test-keyboard test-Keyboard.cpp)
target_link_libraries(test-keyboard  omnimobot eeros)

add_executable(RobotControlTest test-RobotControl.cc)
target_link_libraries(RobotControlTest omnimobot eeros)

add_executable(test-homing test-Homing.cpp)
target_link_libraries(test-homing omnimobot eeros)

add_executable(test-drives test-Drives.cpp)
target_link_libraries(test-drives omnimobot eeros)

add_executable(test-axis5 test-Axis5.cpp)
target_link_libraries(test-axis5 omnimobot eeros)

add_executable(test-nan test-Nan.cpp)
target_link_libraries(test-nan omnimobot eeros)

add_executable(test-control test-ControlSystem.cpp)
target_link_libraries(test-control omnimobot eeros)

add_executable(test-digIn test-DigIn.cpp)
target_link_libraries(test-digIn omnimobot eeros)

add_executable(test-digInWithSafty test-DigInWithSafty.cpp)
target_link_libraries(test-digInWithSafty omnimobot eeros)

add_executable(test-controljacobi test-ControlSystemKartesisch.cpp)
target_link_libraries(test-controljacobi omnimobot eeros)

add_executable(test-controlandSafety test-ControlAndSafety.cpp)
target_link_libraries(test-controlandSafety omnimobot eeros)

add_executable(test-Enc test-HomingSensorAndEnc.cpp)
target_link_libraries(test-Enc eeros)

add_executable(test-Hall test-Hallsensor.cpp)
target_link_libraries(test-Hall eeros)

add_executable(test-atiptophiblock test-AtipToPihiBlock.cpp)
target_link_libraries(test-atiptophiblock eeros omnimobot)

add_executable(test-Hallplot test-Hallplot.cpp)
target_link_libraries(test-Hallplot eeros)

# add_executable(test-Hallblock test-HallBlock.cpp)
# target_link_libraries(test-Hallblock eeros)

add_executable(test-spi test-SPI.cpp)
target_link_libraries(test-spi eeros)

add_executable(test-maxon test-Maxon.cpp)
target_link_libraries(test-maxon eeros)

add_executable(test-mainboard test-Mainboard.cpp)
target_link_libraries(test-mainboard eeros)

# add_executable(test-controlSystem test-ControlSystem.cpp)
# target_link_libraries(test-controlSystem eeros omnimobot)

add_executable(testsaftywithoutcontroller test-SafetyWithoutControlSystem.cpp SafetyWithoutControlSystem.cpp ControlsystemDummy.cpp)
target_link_libraries(testsaftywithoutcontroller eeros omnimobot)

add_executable(subblock test-Subblock.cpp)
target_link_libraries(subblock eeros)

# define test data directory
set(TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/test/data)

# add unit tests
add_test(control/block/i i-test "${TEST_DATA_DIR}/i-test-v02.txt")
add_test(control/block/InvJacobian Inv-jacobian-test "${TEST_DATA_DIR}/inv_jacobian-test.txt")
add_test(control/block/RobotControlblock RobotControlTest "${TEST_DATA_DIR}/RobotControl.txt")