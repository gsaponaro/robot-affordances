# Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Author: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>,
#         Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

# requires minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# produce the cmake var PROJECT_NAME
project(handActions)

# mandatory use of these packages
find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# helpers defining certain macros (e.g. "yarp_install")
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# Thrift IDL
include(YarpIDL)
#yarp_add_idl(IDL_GEN_FILES ${PROJECT_NAME}.thrift)
#set(idl ${PROJECT_NAME}.thrift)
set(idl_files ${PROJECT_NAME}.thrift)
yarp_add_idl(IDL_GEN_FILES ${idl_files})
source_group("IDL Files" FILES ${idl_files})

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${IDL_GENERATED_CODE_DIR}/include)
add_executable(${PROJECT_NAME} src/main.cpp
                               src/HandActionsModule.cpp
                               include/HandActionsModule.h
                               ${IDL_GEN_FILES}
                               ${idl_files})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
#icubcontrib_add_uninstall_target()

#file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES handActions.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})
