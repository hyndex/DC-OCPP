cmake_minimum_required(VERSION 3.16)

project(dc_ocpp LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG OFF)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
add_compile_options(-pthread)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread")
# Force Boost.Log to build with threading enabled so we link against multithreaded symbols
add_compile_definitions(BOOST_HAS_THREADS BOOST_LOG_USE_STD_THREADS BOOST_LOG_WITHOUT_THREADING=0)

include(FetchContent)

set(DISABLE_EDM ON CACHE BOOL "" FORCE)

# Fetch helper macros required by libocpp and dependencies
FetchContent_Declare(
  everest-cmake
  GIT_REPOSITORY https://github.com/EVerest/everest-cmake.git
  GIT_TAG main
)
FetchContent_MakeAvailable(everest-cmake)
list(APPEND CMAKE_PREFIX_PATH ${everest-cmake_SOURCE_DIR})
set("everest-cmake_DIR" "${everest-cmake_SOURCE_DIR}" CACHE PATH "everest-cmake dir" FORCE)
set(everest_cmake_DIR "${everest-cmake_SOURCE_DIR}" CACHE PATH "everest-cmake dir (underscore)" FORCE)

cmake_policy(SET CMP0167 OLD)
set(Boost_NO_BOOST_CMAKE ON)

find_package(date QUIET)
find_package(nlohmann_json QUIET)
find_package(nlohmann_json_schema_validator QUIET)
find_package(everest-timer QUIET)
find_package(everest-log QUIET)
find_package(Threads REQUIRED)
find_package(everest-evse_security QUIET)
find_package(everest-sqlite QUIET)
find_package(Boost REQUIRED COMPONENTS log log_setup filesystem thread regex date_time)
if(NOT date_FOUND OR NOT TARGET date::date-tz)
  message(STATUS "date::date-tz not found, fetching header-only date library")
  set(BUILD_TZ_LIB ON CACHE BOOL "" FORCE)
  set(USE_SYSTEM_TZ_DB ON CACHE BOOL "" FORCE)
  set(ENABLE_DATE_TESTING OFF CACHE BOOL "" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    hh_date
    GIT_REPOSITORY https://github.com/HowardHinnant/date.git
    GIT_TAG v3.0.1
  )
  FetchContent_MakeAvailable(hh_date)
  list(APPEND CMAKE_PREFIX_PATH "${hh_date_SOURCE_DIR}")
  if(NOT DEFINED DATE_INCLUDE_DIR)
    set(DATE_INCLUDE_DIR "${hh_date_SOURCE_DIR}/include" CACHE PATH "date include dir")
  else()
    set(DATE_INCLUDE_DIR "${hh_date_SOURCE_DIR}/include" CACHE PATH "date include dir" FORCE)
  endif()
endif()
if(TARGET date-tz AND NOT TARGET date::date-tz)
  add_library(date::date-tz ALIAS date-tz)
endif()
if(TARGET date::date AND NOT TARGET date::date-tz)
  add_library(date::date-tz ALIAS date::date)
endif()
if(NOT nlohmann_json_FOUND)
  message(STATUS "nlohmann_json not found, fetching header-only library")
  FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
  )
  FetchContent_MakeAvailable(nlohmann_json)
  list(APPEND CMAKE_PREFIX_PATH "${nlohmann_json_SOURCE_DIR}")
  if(NOT DEFINED NLOHMANN_JSON_INCLUDE_DIR)
    set(NLOHMANN_JSON_INCLUDE_DIR "${nlohmann_json_SOURCE_DIR}/include" CACHE PATH "nlohmann json include dir")
  else()
    set(NLOHMANN_JSON_INCLUDE_DIR "${nlohmann_json_SOURCE_DIR}/include" CACHE PATH "nlohmann json include dir" FORCE)
  endif()
endif()
if(NOT nlohmann_json_schema_validator_FOUND)
  message(STATUS "nlohmann_json_schema_validator not found, fetching library")
  FetchContent_Declare(
    nlohmann_json_schema_validator
    GIT_REPOSITORY https://github.com/pboettch/json-schema-validator.git
    GIT_TAG 2.3.0
  )
  set(JSON_VALIDATOR_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(JSON_VALIDATOR_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(nlohmann_json_schema_validator)
  list(APPEND CMAKE_PREFIX_PATH "${nlohmann_json_schema_validator_SOURCE_DIR}")
  if(TARGET nlohmann_json_schema_validator AND NOT TARGET nlohmann_json_schema_validator::nlohmann_json_schema_validator)
    add_library(nlohmann_json_schema_validator::nlohmann_json_schema_validator ALIAS nlohmann_json_schema_validator)
  endif()
  if(NOT DEFINED NLOHMANN_JSON_SCHEMA_VALIDATOR_INCLUDE_DIR)
    set(NLOHMANN_JSON_SCHEMA_VALIDATOR_INCLUDE_DIR "${nlohmann_json_schema_validator_SOURCE_DIR}/src" CACHE PATH "json schema validator include dir")
  else()
    set(NLOHMANN_JSON_SCHEMA_VALIDATOR_INCLUDE_DIR "${nlohmann_json_schema_validator_SOURCE_DIR}/src" CACHE PATH "json schema validator include dir" FORCE)
  endif()
  if(TARGET nlohmann_json_schema_validator)
    set(NLOHMANN_JSON_SCHEMA_VALIDATOR_LIBRARY nlohmann_json_schema_validator CACHE STRING "json schema validator lib path" FORCE)
  elseif(TARGET nlohmann_json_schema_validator::nlohmann_json_schema_validator)
    set(NLOHMANN_JSON_SCHEMA_VALIDATOR_LIBRARY nlohmann_json_schema_validator::nlohmann_json_schema_validator CACHE STRING "json schema validator lib path" FORCE)
  endif()
  set(NLOHMANN_JSON_SCHEMA_VALIDATOR_LIBRARY "${CMAKE_BINARY_DIR}/_deps/nlohmann_json_schema_validator-build/libnlohmann_json_schema_validator.a" CACHE STRING "json schema validator lib path" FORCE)
endif()
unset(LIBWEBSOCKETS_LIBRARY CACHE)
unset(LIBWEBSOCKETS_INCLUDE_DIR CACHE)
if(NOT libwebsockets_FOUND)
  message(STATUS "libwebsockets not found, fetching minimal build")
  FetchContent_Declare(
    libwebsockets
    GIT_REPOSITORY https://github.com/warmcat/libwebsockets.git
    GIT_TAG v4.3.3
  )
  set(LWS_WITH_SHARED OFF CACHE BOOL "" FORCE)
  set(LWS_WITH_STATIC ON CACHE BOOL "" FORCE)
  set(LWS_WITH_SSL ON CACHE BOOL "" FORCE)
  set(LWS_WITH_ZLIB OFF CACHE BOOL "" FORCE)
  set(LWS_WITHOUT_TESTAPPS ON CACHE BOOL "" FORCE)
  set(LWS_WITHOUT_EXTENSIONS OFF CACHE BOOL "" FORCE)
  set(LWS_WITHOUT_CLIENT OFF CACHE BOOL "" FORCE)
  set(LWS_WITHOUT_SERVER OFF CACHE BOOL "" FORCE)
  set(LWS_WITH_HTTP2 ON CACHE BOOL "" FORCE)
  set(LWS_IPV6 ON CACHE BOOL "" FORCE)
  set(LWS_WITH_LIBCAP OFF CACHE BOOL "" FORCE)
  set(LWS_HAVE_SYS_CAPABILITY_H OFF CACHE BOOL "" FORCE)
  set(LWS_WITH_LEJP OFF CACHE BOOL "" FORCE)
  set(LWS_WITH_LEJP_CONF OFF CACHE BOOL "" FORCE)
  set(LWS_WITH_MINIMAL_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(LWS_WITHOUT_BUILTIN_SHA1 ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(libwebsockets)
  list(APPEND CMAKE_PREFIX_PATH "${libwebsockets_SOURCE_DIR}")
  if(TARGET websockets_shared AND NOT TARGET websockets)
    add_library(websockets ALIAS websockets_shared)
  endif()
  if(TARGET websockets_static AND NOT TARGET websockets_shared)
    add_library(websockets_shared ALIAS websockets_static)
  endif()
  if(TARGET websockets AND NOT TARGET websockets_shared)
    add_library(websockets_shared ALIAS websockets)
  endif()
  if(EXISTS "${libwebsockets_BINARY_DIR}/lib/libwebsockets.a")
    set(LIBWEBSOCKETS_LIBRARY "${libwebsockets_BINARY_DIR}/lib/libwebsockets.a" CACHE STRING "libwebsockets library" FORCE)
  elseif(TARGET websockets_shared)
    set(LIBWEBSOCKETS_LIBRARY "$<TARGET_FILE:websockets_shared>" CACHE STRING "libwebsockets library" FORCE)
  endif()
  if(NOT LIBWEBSOCKETS_LIBRARY)
    set(LIBWEBSOCKETS_LIBRARY "websockets_shared" CACHE STRING "libwebsockets library target" FORCE)
  endif()
  if(EXISTS "${libwebsockets_BINARY_DIR}/include")
    set(LIBWEBSOCKETS_INCLUDE_DIR "${libwebsockets_BINARY_DIR}/include" CACHE PATH "libwebsockets include dir" FORCE)
  elseif(TARGET websockets_shared)
    get_target_property(_lws_inc websockets_shared INTERFACE_INCLUDE_DIRECTORIES)
    if(_lws_inc)
      list(GET _lws_inc 0 _lws_inc0)
      set(LIBWEBSOCKETS_INCLUDE_DIR "${_lws_inc0}" CACHE PATH "libwebsockets include dir" FORCE)
    endif()
  endif()
endif()
if(DEFINED libwebsockets_BINARY_DIR AND EXISTS "${libwebsockets_BINARY_DIR}/include")
  set(LIBWEBSOCKETS_INCLUDE_DIR "${libwebsockets_BINARY_DIR}/include" CACHE PATH "libwebsockets include dir" FORCE)
endif()
if(NOT everest-timer_FOUND)
  message(STATUS "everest-timer not found, fetching")
  FetchContent_Declare(
    everest_timer
    GIT_REPOSITORY https://github.com/EVerest/libtimer.git
    GIT_TAG v0.1.3
  )
  FetchContent_MakeAvailable(everest_timer)
  if(TARGET everest-timer)
    set(EVEREST_TIMER_INCLUDE_DIR "${everest_timer_SOURCE_DIR}/include" CACHE PATH "everest timer include dir" FORCE)
  elseif(TARGET everest_timer)
    set(EVEREST_TIMER_INCLUDE_DIR "${everest_timer_SOURCE_DIR}/include" CACHE PATH "everest timer include dir" FORCE)
    add_library(everest-timer ALIAS everest_timer)
  else()
    set(EVEREST_TIMER_INCLUDE_DIR "${everest_timer_SOURCE_DIR}/include" CACHE PATH "everest timer include dir" FORCE)
  endif()
endif()

if(NOT everest-log_FOUND)
  message(STATUS "everest-log not found, fetching")
  FetchContent_Declare(
    everest_log
    GIT_REPOSITORY https://github.com/EVerest/liblog.git
    GIT_TAG v0.3.0
  )
  FetchContent_MakeAvailable(everest_log)
  if(TARGET everest_log AND NOT TARGET everest-log)
    add_library(everest-log ALIAS everest_log)
  endif()
  if(TARGET everest_log)
    set(EVEREST_LOG_INCLUDE_DIR "${everest_log_SOURCE_DIR}/include" CACHE PATH "everest log include dir" FORCE)
    set(EVEREST_LOG_LIBRARY everest_log CACHE STRING "everest log lib path" FORCE)
  endif()
endif()
if(TARGET everest_log)
  target_compile_options(everest_log PUBLIC -pthread)
  target_link_libraries(everest_log PUBLIC Threads::Threads)
endif()

if(NOT everest-evse_security_FOUND)
  message(STATUS "everest-evse_security not found, fetching")
  FetchContent_Declare(
    everest_evse_security
    GIT_REPOSITORY https://github.com/EVerest/libevse-security.git
    GIT_TAG v0.10.0
  )
  FetchContent_MakeAvailable(everest_evse_security)
  if(TARGET everest_evse_security AND NOT TARGET everest-evse_security)
    add_library(everest-evse_security ALIAS everest_evse_security)
  endif()
  if(TARGET evse_security)
    set(EVSE_SECURITY_INCLUDE_DIR "${everest_evse_security_SOURCE_DIR}/include" CACHE PATH "evse security include dir" FORCE)
    set(EVSE_SECURITY_LIBRARY evse_security CACHE STRING "evse security lib path" FORCE)
  elseif(TARGET everest_evse_security)
    set(EVSE_SECURITY_INCLUDE_DIR "${everest_evse_security_SOURCE_DIR}/include" CACHE PATH "evse security include dir" FORCE)
    set(EVSE_SECURITY_LIBRARY everest_evse_security CACHE STRING "evse security lib path" FORCE)
  endif()
endif()
if(TARGET evse_security)
  target_compile_options(evse_security PUBLIC -pthread)
  target_link_libraries(evse_security PUBLIC Threads::Threads)
endif()

if(NOT everest-sqlite_FOUND)
  message(STATUS "everest-sqlite not found, fetching")
  FetchContent_Declare(
    everest_sqlite
    GIT_REPOSITORY https://github.com/EVerest/everest-sqlite.git
    GIT_TAG v0.1.4
  )
  FetchContent_MakeAvailable(everest_sqlite)
  if(TARGET everest_sqlite AND NOT TARGET everest-sqlite)
    add_library(everest-sqlite ALIAS everest_sqlite)
  endif()
  if(TARGET everest_sqlite)
    set(EVEREST_SQLITE_INCLUDE_DIR "${everest_sqlite_SOURCE_DIR}/include" CACHE PATH "everest sqlite include dir" FORCE)
    set(EVEREST_SQLITE_LIBRARY everest_sqlite CACHE STRING "everest sqlite lib path" FORCE)
  endif()
endif()

find_package(CURL REQUIRED)

# Keep libocpp lean for this build
set(LIBOCPP_ENABLE_V16 ON CACHE BOOL "" FORCE)
set(LIBOCPP_ENABLE_V2 OFF CACHE BOOL "" FORCE)
set(LIBOCPP_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(LIBOCPP16_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

add_subdirectory(libocpp)

if(TARGET ocpp AND DEFINED libwebsockets_BINARY_DIR AND EXISTS "${libwebsockets_BINARY_DIR}/include")
  target_include_directories(ocpp SYSTEM PUBLIC "${libwebsockets_BINARY_DIR}/include")
endif()
if(TARGET ocpp)
  target_compile_options(ocpp PUBLIC -pthread)
  target_link_libraries(ocpp PUBLIC Threads::Threads)
endif()

add_executable(dc_ocpp
  src/main.cpp
  src/charger_config.cpp
  src/ocpp_adapter.cpp
  src/hardware_sim.cpp
  src/can_plc.cpp
  src/power_manager.cpp
)

add_custom_target(evse_security_build
  COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR}/_deps/everest_evse_security-build --target evse_security
  BYPRODUCTS ${EVSE_SECURITY_LIBRARY}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/_deps/everest_evse_security-build
)

target_include_directories(dc_ocpp PRIVATE include)
add_dependencies(dc_ocpp evse_security_build)

# Prefer CMake targets for dependencies when available to avoid stale archive paths
set(EVEREST_LOG_LIBRARY "${CMAKE_BINARY_DIR}/_deps/everest_log-build/lib/libeverest_log.a" CACHE STRING "everest log lib path" FORCE)
if(NOT TARGET everest_log AND EXISTS "${EVEREST_LOG_LIBRARY}")
  add_custom_target(everest_log DEPENDS "${EVEREST_LOG_LIBRARY}")
endif()
set(_EVEREST_LOG_DEP ${EVEREST_LOG_LIBRARY})

set(NLOHMANN_JSON_SCHEMA_VALIDATOR_LIBRARY "${CMAKE_BINARY_DIR}/_deps/nlohmann_json_schema_validator-build/libnlohmann_json_schema_validator.a" CACHE STRING "json schema validator lib path" FORCE)
set(_JSON_SCHEMA_DEP ${NLOHMANN_JSON_SCHEMA_VALIDATOR_LIBRARY})
if(TARGET nlohmann_json_schema_validator::nlohmann_json_schema_validator)
  set(_JSON_SCHEMA_DEP nlohmann_json_schema_validator::nlohmann_json_schema_validator)
elseif(TARGET nlohmann_json_schema_validator)
  set(_JSON_SCHEMA_DEP nlohmann_json_schema_validator)
endif()
set(_JSON_SCHEMA_ARCHIVE "${CMAKE_BINARY_DIR}/_deps/nlohmann_json_schema_validator-build/libnlohmann_json_schema_validator.a")
if(EXISTS "${_JSON_SCHEMA_ARCHIVE}")
  set(_JSON_SCHEMA_DEP "${_JSON_SCHEMA_ARCHIVE}")
endif()
if(NOT TARGET nlohmann_json_schema_validator)
  add_custom_target(nlohmann_json_schema_validator DEPENDS "${_JSON_SCHEMA_ARCHIVE}")
endif()

set(_EVSE_SECURITY_DEP ${EVSE_SECURITY_LIBRARY})
set(EVSE_SECURITY_LIBRARY "${CMAKE_BINARY_DIR}/_deps/everest_evse_security-build/lib/evse_security/libevse_security.a" CACHE STRING "evse security lib path" FORCE)
if(NOT TARGET evse_security)
  if(EXISTS "${EVSE_SECURITY_LIBRARY}")
    add_library(evse_security STATIC IMPORTED)
    set_target_properties(evse_security PROPERTIES IMPORTED_LOCATION "${EVSE_SECURITY_LIBRARY}")
  else()
    add_custom_target(evse_security DEPENDS "${EVSE_SECURITY_LIBRARY}")
  endif()
endif()
set(_EVSE_SECURITY_DEP ${EVSE_SECURITY_LIBRARY})

set(_EVEREST_SQLITE_DEP ${EVEREST_SQLITE_LIBRARY})
if(TARGET everest_sqlite)
  set(_EVEREST_SQLITE_DEP everest_sqlite)
elseif(TARGET everest-sqlite)
  set(_EVEREST_SQLITE_DEP everest-sqlite)
endif()

target_link_libraries(dc_ocpp PRIVATE
  everest::ocpp
  date::date-tz
  CURL::libcurl
  ${_EVEREST_LOG_DEP}
  ${_JSON_SCHEMA_DEP}
  ${_EVSE_SECURITY_DEP}
  ${_EVEREST_SQLITE_DEP}
  Boost::log
  Boost::log_setup
  Boost::filesystem
  Boost::thread
  Boost::regex
  Boost::date_time
  Threads::Threads
)

add_executable(power_manager_tests
  tests/power_manager_tests.cpp
  src/power_manager.cpp
)
target_include_directories(power_manager_tests PRIVATE include)
target_link_libraries(power_manager_tests PRIVATE pthread)

add_executable(power_manager_invariants_tests
  tests/power_manager_invariants_tests.cpp
  src/power_manager.cpp
)
target_include_directories(power_manager_invariants_tests PRIVATE include)
target_link_libraries(power_manager_invariants_tests PRIVATE pthread)

add_executable(can_crc_filter_tests
  tests/can_crc_filter_tests.cpp
)
target_include_directories(can_crc_filter_tests PRIVATE include libocpp/include)
target_link_libraries(can_crc_filter_tests PRIVATE nlohmann_json::nlohmann_json date::date-tz)
