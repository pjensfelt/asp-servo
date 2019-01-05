# - Try to find SOEM
# Once done this will define
#  SOEM_FOUND - System has SOEM
#  SOEM_INCLUDE_DIRS - The SOEM include directories
#  SOEM_LIBRARIES - The libraries needed to use libsoem
#  OEM_DEFINITIONS - Compiler switches required for using libsoem

find_path(SOEM_INCLUDE_DIR ethercat.h
          HINTS /usr/local/include
          PATH_SUFFIXES soem)

#message("SOEM_INCLUDE_DIR: ${SOEM_INCLUDE_DIR}")


find_library(SOEM_LIBRARY NAMES libsoem.so
             HINTS /usr/local/lib )

#message("SOEM_LIBRARY: ${SOEM_LIBRARY}")

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set SOEM_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(SOEM  DEFAULT_MSG
                                  SOEM_LIBRARY SOEM_INCLUDE_DIR)

mark_as_advanced(SOEM_INCLUDE_DIR SOEM_LIBRARY )

set(SOEM_LIBRARIES ${SOEM_LIBRARY} )
set(SOEM_INCLUDE_DIRS ${SOEM_INCLUDE_DIR} )
