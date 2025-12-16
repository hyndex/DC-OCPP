# Raspberry Pi cross/toolchain settings.
#
# Usage examples:
#   cmake -S . -B build-rpi -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/raspberrypi.cmake
#   cmake -S . -B build-rpi -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/raspberrypi.cmake -DRPI_ARCH=armhf

set(CMAKE_SYSTEM_NAME Linux)

set(RPI_ARCH "arm64" CACHE STRING "Target Raspberry Pi arch (arm64 or armhf)")
set_property(CACHE RPI_ARCH PROPERTY STRINGS arm64 armhf)

if(RPI_ARCH STREQUAL "armhf")
  set(RPI_TRIPLE "arm-linux-gnueabihf")
  set(CMAKE_SYSTEM_PROCESSOR arm)
  set(RPI_COMMON_FLAGS "-march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard")
else()
  set(RPI_TRIPLE "aarch64-linux-gnu")
  set(CMAKE_SYSTEM_PROCESSOR aarch64)
  set(RPI_COMMON_FLAGS "-march=armv8-a")
endif()

if(NOT DEFINED CMAKE_SYSROOT)
  if(EXISTS "/usr/${RPI_TRIPLE}")
    set(CMAKE_SYSROOT "/usr/${RPI_TRIPLE}")
  elseif(EXISTS "/usr/${RPI_TRIPLE}/libc")
    set(CMAKE_SYSROOT "/usr/${RPI_TRIPLE}/libc")
  endif()
endif()

set(CMAKE_C_COMPILER "${RPI_TRIPLE}-gcc")
set(CMAKE_CXX_COMPILER "${RPI_TRIPLE}-g++")

set(_RPI_FIND_ROOTS)
if(CMAKE_SYSROOT)
  list(APPEND _RPI_FIND_ROOTS "${CMAKE_SYSROOT}")
endif()
list(APPEND _RPI_FIND_ROOTS "/usr/${RPI_TRIPLE}" "/usr/lib/${RPI_TRIPLE}")
set(CMAKE_FIND_ROOT_PATH ${_RPI_FIND_ROOTS})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

add_compile_options(${RPI_COMMON_FLAGS})

# Keep pkg-config pointed at the target architecture when present
if(DEFINED ENV{PKG_CONFIG_PATH})
  set(ENV{PKG_CONFIG_PATH} "/usr/lib/${RPI_TRIPLE}/pkgconfig:/usr/share/pkgconfig:$ENV{PKG_CONFIG_PATH}")
else()
  set(ENV{PKG_CONFIG_PATH} "/usr/lib/${RPI_TRIPLE}/pkgconfig:/usr/share/pkgconfig")
endif()
