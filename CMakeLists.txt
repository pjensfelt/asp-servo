cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(asp_servo_api)
 
set(CMAKE_CXX_FLAGS -std=c++11)

# To allow using asp_servo_api in another shared library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(asp_servo_api_LIB_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(asp_servo_api_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/asp_servo_api)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)

add_library(asp_servo_api
    src/asp_servo.cpp
    src/servo.cpp
)
target_link_libraries(asp_servo_api pthread rt soem tinyxml2)

file(GLOB asp_servo_api_HEADERS include/asp_servo_api/*.h)

message("asp_servo_api_LIB_INSTALL_DIR: ${asp_servo_api_LIB_INSTALL_DIR}")
message("asp_servo_api_INCLUDE_INSTALL_DIR: ${asp_servi_api_INCLUDE_INSTALL_DIR}")

# Install library
install(TARGETS asp_servo_api DESTINATION ${asp_servo_api_LIB_INSTALL_DIR})

# Install header files
install(FILES
  ${asp_servo_api_HEADERS}
  DESTINATION ${asp_servo_api_INCLUDE_INSTALL_DIR})

# uninstall target
if(NOT TARGET uninstall)
  configure_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
      "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
      IMMEDIATE @ONLY)

  add_custom_target(uninstall
      COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
