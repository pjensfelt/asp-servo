cmake_minimum_required(VERSION 3.5)

project(asp_servo_api VERSION 0.1 LANGUAGES CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# To allow using tinyxml in another shared library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#set()
#find_package(libsoem REQUIRED )

#Temporary, compile with debug info
set(CMAKE_BUILD_TYPE Debug)

option(BUILD_SHARED_LIBS "build as shared library" ON)
option(BUILD_STATIC_LIBS "build as static library" OFF)

add_library(asp_servo_api
    src/asp_servo.cpp 
    src/servo.cpp
)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(asp_servo_api 
    PUBLIC
       $<INSTALL_INTERFACE:include>
       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE 
       ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(asp_servo_api soem pthread rt tinyxml2)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS asp_servo_api 
    EXPORT asp_servo_apiTargets
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
) 

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT asp_servo_apiTargets
    FILE 
        asp_servo_apiTargets.cmake
    NAMESPACE 
        asp_servo_api::
    DESTINATION 
        ${CMAKE_INSTALL_LIBDIR}/cmake/asp_servo_api
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
       ${CMAKE_CURRENT_BINARY_DIR}/asp_servo_apiConfigVersion.cmake
    VERSION 
       ${PROJECT_VERSION}
    COMPATIBILITY 
       AnyNewerVersion
)

include(CMakePackageConfigHelpers)
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
configure_package_config_file(
  "Config.cmake.in"
  "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${CMAKE_PROJECT_NAME}"
)
install(FILES
        ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${CMAKE_PROJECT_NAME})

install(EXPORT ${CMAKE_PROJECT_NAME}Targets NAMESPACE asp_servo_api::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${CMAKE_PROJECT_NAME})


# uninstall target
if(NOT TARGET uninstall)
  configure_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
      "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
      IMMEDIATE @ONLY)

  add_custom_target(uninstall
      COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
