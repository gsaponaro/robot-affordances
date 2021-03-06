# Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
#         Atabak Dehban <adehban@isr.tecnico.ulisboa.pt>,
#         Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>,
#         Lorenzo Jamone, Afonso Gonçalves
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

# requires minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# produce the cmake var PROJECT_NAME
project(handAffManager)

# mandatory use of these packages
find_package(YARP REQUIRED COMPONENTS OS idl_tools sig)
find_package(ICUBcontrib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED filesystem system)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# helpers defining certain macros
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# Thrift IDL
set(idl_files ${PROJECT_NAME}.thrift)
yarp_add_idl(IDL_GEN_FILES ${idl_files})
source_group("IDL Files" FILES ${idl_files})

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/../common/include
                    ${OpenCV_INCLUDE_DIRS}
                    ${IDL_GENERATED_CODE_DIR}/include
                    ${Boost_FILESYSTEM_INCLUDE_DIRS}
                    ${Boost_SYSTEM_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/HandAffManagerMain.cpp
                               src/HandAffManagerModule.cpp
                               include/HandAffManagerModule.h
                               include/CSVFile.h
                               ${PROJECT_SOURCE_DIR}/../common/include/OpenCVHelpers.h
                               ${IDL_GEN_FILES}
                               ${idl_files})
target_link_libraries(${PROJECT_NAME}
                      ${YARP_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${Boost_FILESYSTEM_LIBRARY}
                      ${Boost_SYSTEM_LIBRARY})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
#icubcontrib_add_uninstall_target()

#file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
#yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
