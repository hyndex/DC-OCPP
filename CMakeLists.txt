cmake_minimum_required(VERSION 3.16)

project(dc_ocpp LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG OFF)
add_definitions(-DLWS_HAVE_SYS_CAPABILITY_H=0 -DLWS_HAVE_LIBCAP=0)

find_package(date REQUIRED)

include(FetchContent)

set(DISABLE_EDM ON CACHE BOOL "" FORCE)

# Fetch helper macros required by libocpp
FetchContent_Declare(
  everest-cmake
  GIT_REPOSITORY https://github.com/EVerest/everest-cmake.git
  GIT_TAG main
)
FetchContent_MakeAvailable(everest-cmake)
list(APPEND CMAKE_PREFIX_PATH ${everest-cmake_SOURCE_DIR})

# Keep libocpp lean for this build
set(LIBOCPP_ENABLE_V16 ON CACHE BOOL "" FORCE)
set(LIBOCPP_ENABLE_V2 OFF CACHE BOOL "" FORCE)
set(LIBOCPP_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(LIBOCPP16_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

add_subdirectory(libocpp)

add_executable(dc_ocpp
  src/main.cpp
  src/charger_config.cpp
  src/ocpp_adapter.cpp
  src/hardware_sim.cpp
  src/can_plc.cpp
  src/power_manager.cpp
)

target_include_directories(dc_ocpp PRIVATE include)
target_link_libraries(dc_ocpp PRIVATE everest::ocpp date::date-tz)

add_executable(power_manager_tests
  tests/power_manager_tests.cpp
  src/power_manager.cpp
)
target_include_directories(power_manager_tests PRIVATE include)
