cmake_minimum_required(VERSION 2.8)

project(omnimobot)

find_package(EEROS REQUIRED)
include_directories(${EEROS_INCLUDE_DIR};${EEROS_LIBUCL_INCLUDE_DIR})
link_directories(${EEROS_LIB_DIR};${EEROS_LIBUCL_LINK_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# includes
include_directories(${ADDITIONAL_INCLUDE_DIRS} include ../laserscanner)
link_directories(${ADDITIONAL_LINK_DIRS} "${CMAKE_BINARY_DIR}/src")

# sources
add_subdirectory(src)
# enable_testing()
# add_subdirectory(test)
