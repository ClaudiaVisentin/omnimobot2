cmake_minimum_required(VERSION 2.8)
project(laserscanner)

find_package(EEROS REQUIRED)
include_directories(${EEROS_INCLUDE_DIR};${EEROS_LIBUCL_INCLUDE_DIR})
link_directories(${EEROS_LIB_DIR};${EEROS_LIBUCL_LINK_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${ADDITIONAL_INCLUDE_DIRS} include ../omnimobot)

add_library(laserscanner
			src/LSData.cpp
			src/LaserScanner.cpp
			src/CollisionDetection.cpp
			src/SafetyVelocityDesired.cpp
)

target_link_libraries(laserscanner eeros ucl ${CMAKE_DL_LIBS})

enable_testing()
add_subdirectory(test)


