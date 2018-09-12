# Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.9)
project(shapeDescriptor)

find_package(YARP REQUIRED COMPONENTS OS idl_tools)
find_package(ICUB)
find_package(ICUBcontrib)

list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

find_package(OpenCV)

include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

file(GLOB source src/*.cpp ../common/src/*.cpp)
file(GLOB header include/*.h ../common/include/*.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/../common/include
                    ${ICUB_INCLUDE_DIRS})

# import math symbols from standard cmath
add_definitions(-D_USE_MATH_DEFINES)

add_executable(${PROJECT_NAME} ${source} ${header})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
yarp_install(FILES shapeDescriptor.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})
