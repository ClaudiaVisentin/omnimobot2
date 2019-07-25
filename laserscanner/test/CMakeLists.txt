add_executable(testScannerVV testSensorVV.cpp)
target_link_libraries(testScannerVV laserscanner eeros)

add_executable(testSensorNeu testNeu.cpp)
target_link_libraries(testSensorNeu laserscanner )

add_executable(testCommand testCommdURG04LX.cpp)
target_link_libraries(testCommand laserscanner eeros)

add_executable(testCollData testCollisionsData.cpp)
target_link_libraries(testCollData laserscanner eeros)

add_executable(testDistance384 testDistanzeStep384.cpp)

add_executable(testlsdata testLSData.cpp)
target_link_libraries(testlsdata laserscanner eeros)

add_executable(testlaserscanner testLaserScanner.cpp)
target_link_libraries(testlaserscanner laserscanner eeros)

add_executable(scanDaten testScannerDaten.cpp)
target_link_libraries(scanDaten laserscanner eeros)



