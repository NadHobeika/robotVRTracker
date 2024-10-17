cmake_minimum_required(VERSION 3.0.2)

project(nadineVR)

if(NOT PLATFORM)
  if(CMAKE_SIZEOF_VOID_P MATCHES 8)
    set(PLATFORM 64)
  else()
    set(PLATFORM 32)
  endif()
endif()
set(WINDOWS_PATH_SUFFIXES win${PLATFORM} Win${PLATFORM} x${PLATFORM})

## OpenVR API path
find_library(OPENVR_LIBRARIES
  NAMES
    openvr_api
  PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/../openvr/bin
    ${CMAKE_CURRENT_SOURCE_DIR}/../openvr/lib
  PATH_SUFFIXES
    osx32
    linux64
    ${WINDOWS_PATH_SUFFIXES}
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)
set(OPENVR_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../openvr/headers)


# -----------------------------------------------------------------------------
## SHARED SOURCES ##

include_directories(
  .
  ${OPENVR_INCLUDE_DIR}
)

add_executable(milouTracker milouTracker.cpp)
target_link_libraries(milouTracker ${OPENVR_LIBRARIES})