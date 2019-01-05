cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)

project(asp_servo_api)
 
##############################################
# Build 3rd party software
ExternalProject_Add(
    tinyxml2
    
    UPDATE_COMMAND ""

    PATCH_COMMAND ""

    SOURCE_DIR "${CMAKE_SOURCE_DIR}/tinyxml2"

    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}/
  
    TEST_COMMAND ""        
)

#set(tinyxml2_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/tinyxml2/include")
#set(tinyxml2_LIBRARIES "${CMAKE_SHARED_LIBRARY_PREFIX}/tinyxml2${CMAKE_SHARED_LIBRARY_SUFFIX}")
#include_directories(${tinyxml2_INCLUDE_DIRS})

ExternalProject_Add(
    SOEM
    
    UPDATE_COMMAND ""

    PATCH_COMMAND ""

    SOURCE_DIR "${CMAKE_SOURCE_DIR}/SOEM"

    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}/
  
    TEST_COMMAND ""        
)

#set(SOEM_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/SOEM/include")
#set(SOEM_LIBRARIES "${CMAKE_SHARED_LIBRARY_PREFIX}/SOEM${CMAKE_SHARED_LIBRARY_SUFFIX}")
#include_directories(${SOEM_INCLUDE_DIRS})

###################################################
# Build the api
ExternalProject_Add(
    asp_servo_api
    
    UPDATE_COMMAND ""

    PATCH_COMMAND ""

    SOURCE_DIR "${CMAKE_SOURCE_DIR}/api"

    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}/
  
    TEST_COMMAND ""        
)

