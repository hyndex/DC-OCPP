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
add_compile_definitions(BOOST_HAS_THREADS BOOST_LOG_USE_STD_THREADS BOOST_LOG_WITHOUT_THREADING=0 BOOST_LOG_DYN_LINK)

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
set(Boost_NO_WARN_NEW_VERSIONS ON)

find_package(date QUIET)
find_package(nlohmann_json QUIET)
set(nlohmann_json_schema_validator_FOUND FALSE)
unset(NLOHMANN_JSON_SCHEMA_VALIDATOR_LIBRARY CACHE)
unset(NLOHMANN_JSON_SCHEMA_VALIDATOR_INCLUDE_DIR CACHE)
find_package(everest-timer QUIET)
find_package(everest-log QUIET)
find_package(Threads REQUIRED)
set(everest-evse_security_FOUND FALSE)
unset(EVSE_SECURITY_LIBRARY CACHE)
unset(EVSE_SECURITY_INCLUDE_DIR CACHE)
set(everest-sqlite_FOUND FALSE)
unset(EVEREST_SQLITE_LIBRARY CACHE)
unset(EVEREST_SQLITE_INCLUDE_DIR CACHE)
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
if(DEFINED DATE_INCLUDE_DIR)
  include_directories("${DATE_INCLUDE_DIR}")
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
if(NOT nlohmann_json_schema_validator_FOUND
    OR (NOT TARGET nlohmann_json_schema_validator AND NOT TARGET nlohmann_json_schema_validator::nlohmann_json_schema_validator)
    OR (DEFINED NLOHMANN_JSON_SCHEMA_VALIDATOR_LIBRARY AND NOT EXISTS "${NLOHMANN_JSON_SCHEMA_VALIDATOR_LIBRARY}"))
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
endif()

if(NOT DEFINED EVEREST_TIMER_PATCHED)
  set(_everest_timer_patch "${CMAKE_CURRENT_SOURCE_DIR}/cmake/patches/everest_timer_duration_cast.patch")
  set(_everest_timer_src "${everest_timer_SOURCE_DIR}")
  if(NOT _everest_timer_src AND DEFINED everest-timer_SOURCE_DIR)
    set(_everest_timer_src "${everest-timer_SOURCE_DIR}")
  endif()
  if(NOT _everest_timer_src)
    set(_everest_timer_src "${CMAKE_BINARY_DIR}/_deps/everest_timer-src")
  endif()
  set(_everest_timer_header "${_everest_timer_src}/include/everest/timer.hpp")
  if(EXISTS "${_everest_timer_patch}" AND EXISTS "${_everest_timer_header}")
    file(READ "${_everest_timer_header}" _everest_timer_hpp)
    if(NOT _everest_timer_hpp MATCHES "interval_duration")
      execute_process(
        COMMAND patch -p1 -i "${_everest_timer_patch}"
        WORKING_DIRECTORY "${_everest_timer_src}"
        RESULT_VARIABLE _everest_timer_patch_result
        COMMAND_ERROR_IS_FATAL ANY
      )
    endif()
    set(EVEREST_TIMER_PATCHED TRUE CACHE INTERNAL "everest timer patch applied")
  endif()
endif()

set(_everest_timer_src_path "${everest_timer_SOURCE_DIR}")
if(NOT _everest_timer_src_path AND DEFINED everest-timer_SOURCE_DIR)
  set(_everest_timer_src_path "${everest-timer_SOURCE_DIR}")
endif()
if(NOT _everest_timer_src_path)
  set(_everest_timer_src_path "${CMAKE_BINARY_DIR}/_deps/everest_timer-src")
endif()
if(TARGET everest-timer)
  set(EVEREST_TIMER_INCLUDE_DIR "${_everest_timer_src_path}/include" CACHE PATH "everest timer include dir" FORCE)
elseif(TARGET everest_timer)
  set(EVEREST_TIMER_INCLUDE_DIR "${_everest_timer_src_path}/include" CACHE PATH "everest timer include dir" FORCE)
  add_library(everest-timer ALIAS everest_timer)
else()
  set(EVEREST_TIMER_INCLUDE_DIR "${_everest_timer_src_path}/include" CACHE PATH "everest timer include dir" FORCE)
endif()
if(TARGET everest::timer)
  get_target_property(_everest_timer_alias everest::timer ALIASED_TARGET)
  if(_everest_timer_alias)
    set_target_properties(${_everest_timer_alias} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_everest_timer_src_path}/include")
  else()
    set_target_properties(everest::timer PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_everest_timer_src_path}/include")
  endif()
endif()

set(_EVEREST_LOG_MISSING TRUE)
if(DEFINED EVEREST_LOG_LIBRARY AND EXISTS "${EVEREST_LOG_LIBRARY}")
  set(_EVEREST_LOG_MISSING FALSE)
endif()
if(NOT everest-log_FOUND OR _EVEREST_LOG_MISSING)
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
  if(DEFINED Boost_INCLUDE_DIRS)
    target_include_directories(everest_log PUBLIC ${Boost_INCLUDE_DIRS})
  endif()
  if(NOT DEFINED EVEREST_LOG_INCLUDE_DIR)
    get_target_property(_everest_log_includes everest_log INTERFACE_INCLUDE_DIRECTORIES)
    if(_everest_log_includes)
      list(GET _everest_log_includes 0 EVEREST_LOG_INCLUDE_DIR)
      set(EVEREST_LOG_INCLUDE_DIR "${EVEREST_LOG_INCLUDE_DIR}" CACHE PATH "everest log include dir" FORCE)
    endif()
  endif()
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
  set(_EVSE_BUILD_LIB "${everest_evse_security_BINARY_DIR}/lib/evse_security")
  if(EXISTS "${CMAKE_BINARY_DIR}/_deps/everest_evse_security-build/lib/evse_security")
    set(EVSE_SECURITY_LIBRARY "${CMAKE_BINARY_DIR}/_deps/everest_evse_security-build/lib/evse_security" CACHE STRING "evse security lib path" FORCE)
    set(EVSE_SECURITY_INCLUDE_DIR "${everest_evse_security_SOURCE_DIR}/include" CACHE PATH "evse security include dir" FORCE)
  elseif(EXISTS "${CMAKE_BINARY_DIR}/_deps/everest_evse_security-build/lib/evse_security/libevse_security.a")
    set(EVSE_SECURITY_LIBRARY "${CMAKE_BINARY_DIR}/_deps/everest_evse_security-build/lib/evse_security/libevse_security.a" CACHE STRING "evse security lib path" FORCE)
    set(EVSE_SECURITY_INCLUDE_DIR "${everest_evse_security_SOURCE_DIR}/include" CACHE PATH "evse security include dir" FORCE)
  endif()
  if(TARGET evse_security)
    set(EVSE_SECURITY_INCLUDE_DIR "${everest_evse_security_SOURCE_DIR}/include" CACHE PATH "evse security include dir" FORCE)
    set(EVSE_SECURITY_LIBRARY evse_security CACHE STRING "evse security lib path" FORCE)
  elseif(TARGET everest_evse_security)
    set(EVSE_SECURITY_INCLUDE_DIR "${everest_evse_security_SOURCE_DIR}/include" CACHE PATH "evse security include dir" FORCE)
    if(EXISTS "${_EVSE_BUILD_LIB}")
      set(EVSE_SECURITY_LIBRARY "${_EVSE_BUILD_LIB}" CACHE STRING "evse security lib path" FORCE)
    else()
      set(EVSE_SECURITY_LIBRARY everest_evse_security CACHE STRING "evse security lib path" FORCE)
    endif()
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
endif()
set(_everest_sqlite_src "${everest_sqlite_SOURCE_DIR}")
if(NOT _everest_sqlite_src AND DEFINED everest-sqlite_SOURCE_DIR)
  set(_everest_sqlite_src "${everest-sqlite_SOURCE_DIR}")
endif()
if(NOT _everest_sqlite_src)
  set(_everest_sqlite_src "${CMAKE_BINARY_DIR}/_deps/everest_sqlite-src")
endif()
set(_everest_sqlite_archive "${CMAKE_BINARY_DIR}/_deps/everest_sqlite-build/lib/libeverest_sqlite.a")
if(NOT DEFINED EVEREST_SQLITE_INCLUDE_DIR)
  set(EVEREST_SQLITE_INCLUDE_DIR "${_everest_sqlite_src}/include" CACHE PATH "everest sqlite include dir" FORCE)
endif()
if(EXISTS "${_everest_sqlite_archive}")
  set(EVEREST_SQLITE_LIBRARY "${_everest_sqlite_archive}" CACHE STRING "everest sqlite lib path" FORCE)
elseif(TARGET everest_sqlite)
  set(EVEREST_SQLITE_LIBRARY everest_sqlite CACHE STRING "everest sqlite lib path" FORCE)
elseif(TARGET everest-sqlite)
  set(EVEREST_SQLITE_LIBRARY everest-sqlite CACHE STRING "everest sqlite lib path" FORCE)
endif()
if(DEFINED Boost_INCLUDE_DIRS)
  if(TARGET everest_sqlite)
    target_include_directories(everest_sqlite PUBLIC ${Boost_INCLUDE_DIRS})
  elseif(TARGET everest-sqlite)
    target_include_directories(everest-sqlite PUBLIC ${Boost_INCLUDE_DIRS})
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
  src/power_module_controller.cpp
  src/can_plc.cpp
  src/power_manager.cpp
)

target_include_directories(dc_ocpp PRIVATE include)

# Prefer CMake targets for dependencies when available to avoid stale archive paths
set(_everest_log_archive "${CMAKE_BINARY_DIR}/_deps/everest_log-build/lib/libeverest_log.a")
set(EVEREST_LOG_LIBRARY "${_everest_log_archive}" CACHE STRING "everest log lib path" FORCE)

set(_JSON_SCHEMA_DEP )
set(_JSON_SCHEMA_ARCHIVE "${CMAKE_BINARY_DIR}/_deps/nlohmann_json_schema_validator-build/libnlohmann_json_schema_validator.a")
if(TARGET nlohmann_json_schema_validator::nlohmann_json_schema_validator)
  set(_JSON_SCHEMA_DEP nlohmann_json_schema_validator::nlohmann_json_schema_validator)
elseif(TARGET nlohmann_json_schema_validator)
  set(_JSON_SCHEMA_DEP nlohmann_json_schema_validator)
elseif(EXISTS "${_JSON_SCHEMA_ARCHIVE}")
  set(_JSON_SCHEMA_DEP "${_JSON_SCHEMA_ARCHIVE}")
  set(NLOHMANN_JSON_SCHEMA_VALIDATOR_LIBRARY "${_JSON_SCHEMA_ARCHIVE}" CACHE STRING "json schema validator lib path" FORCE)
endif()

set(_evse_security_archive "${CMAKE_BINARY_DIR}/_deps/everest_evse_security-build/lib/evse_security/libevse_security.a")
set(EVSE_SECURITY_LIBRARY "${_evse_security_archive}" CACHE STRING "evse security lib path" FORCE)
set(_EVSE_SECURITY_DEP "${_evse_security_archive}")
if(TARGET everest_evse_security)
  set(_EVSE_SECURITY_DEP everest_evse_security)
elseif(TARGET evse_security)
  set(_EVSE_SECURITY_DEP evse_security)
elseif(TARGET everest::evse_security)
  set(_EVSE_SECURITY_DEP everest::evse_security)
endif()

set(_EVEREST_SQLITE_DEP ${EVEREST_SQLITE_LIBRARY})
if(EXISTS "${_everest_sqlite_archive}")
  set(_EVEREST_SQLITE_DEP "${_everest_sqlite_archive}")
endif()
if(TARGET everest_sqlite)
  set(_EVEREST_SQLITE_DEP everest_sqlite)
elseif(TARGET everest-sqlite)
  set(_EVEREST_SQLITE_DEP everest-sqlite)
endif()

target_link_libraries(dc_ocpp PRIVATE
  everest::ocpp
  date::date-tz
  CURL::libcurl
  ${_JSON_SCHEMA_DEP}
  ${_EVEREST_SQLITE_DEP}
  Boost::log
  Boost::log_setup
  Boost::filesystem
  Boost::thread
  Boost::regex
  Boost::date_time
  Threads::Threads
)

option(BUILD_CONFIG_WEB_UI "Build the webview-powered config editor" ON)

if(BUILD_CONFIG_WEB_UI)
  set(_CAN_BUILD_WEBUI TRUE)
  set(_WEBKIT_LIBS "")
  set(_WEBKIT_INCLUDES "")
  set(_USE_OBJCXX FALSE)
  set(WEBKIT_FOUND FALSE)
  set(GTK3_FOUND FALSE)
  set(DNSSD_FOUND FALSE)

  if(APPLE)
    find_library(COCOA_FRAMEWORK Cocoa)
    find_library(FOUNDATION_FRAMEWORK Foundation)
    find_library(WEBKIT_FRAMEWORK WebKit)
    if(COCOA_FRAMEWORK AND FOUNDATION_FRAMEWORK AND WEBKIT_FRAMEWORK)
      list(APPEND _WEBKIT_LIBS ${COCOA_FRAMEWORK} ${FOUNDATION_FRAMEWORK} ${WEBKIT_FRAMEWORK})
      set(_USE_OBJCXX TRUE)
    else()
      set(_CAN_BUILD_WEBUI FALSE)
      message(WARNING "Config web UI disabled: WebKit/Cocoa frameworks not found")
    endif()
  elseif(UNIX)
    find_package(PkgConfig)
    if(PKG_CONFIG_FOUND)
      set(_WEBKIT_PKGS webkit2gtk-4.1 webkit2gtk-4.0)
      foreach(_pkg ${_WEBKIT_PKGS})
        pkg_check_modules(WEBKIT QUIET ${_pkg})
        if(WEBKIT_FOUND)
          break()
        endif()
      endforeach()
      pkg_check_modules(GTK3 QUIET gtk+-3.0)
      if(WEBKIT_FOUND AND GTK3_FOUND)
        list(APPEND _WEBKIT_INCLUDES ${WEBKIT_INCLUDE_DIRS} ${GTK3_INCLUDE_DIRS})
        list(APPEND _WEBKIT_LIBS ${WEBKIT_LIBRARIES} ${GTK3_LIBRARIES})
      else()
        set(_CAN_BUILD_WEBUI FALSE)
        message(WARNING "Config web UI disabled: webkit2gtk or gtk3 dev packages not found")
      endif()
    else()
      set(_CAN_BUILD_WEBUI FALSE)
      message(WARNING "Config web UI disabled: pkg-config not available")
    endif()
  elseif(WIN32)
    list(APPEND _WEBKIT_LIBS ole32 version shlwapi)
  endif()

  find_path(DNSSD_INCLUDE_DIR dns_sd.h)
  find_library(DNSSD_LIBRARY dns_sd)
  if(DNSSD_INCLUDE_DIR AND DNSSD_LIBRARY)
    set(DNSSD_FOUND TRUE)
  endif()

  if(_CAN_BUILD_WEBUI)
    if(APPLE AND _USE_OBJCXX)
      enable_language(OBJCXX)
    endif()
    add_executable(config_webui
      src/config_webui.cpp
      src/charger_config.cpp
    )
    target_include_directories(config_webui PRIVATE include third_party/webview/include third_party third_party/webview ${_WEBKIT_INCLUDES} ${DNSSD_INCLUDE_DIR})
    target_link_libraries(config_webui PRIVATE Threads::Threads ${_WEBKIT_LIBS} nlohmann_json::nlohmann_json)
    if(DNSSD_FOUND)
      target_link_libraries(config_webui PRIVATE ${DNSSD_LIBRARY})
      target_compile_definitions(config_webui PRIVATE HAVE_DNSSD)
    else()
      message(WARNING "mDNS (dns_sd.h) not found; --mdns disabled")
    endif()
    if(WIN32)
      target_link_libraries(config_webui PRIVATE ws2_32)
    endif()
    target_compile_definitions(config_webui PRIVATE
      CONFIG_WEBUI_ASSET_DIR="${CMAKE_SOURCE_DIR}/webui"
      CONFIG_WEBUI_DEFAULT_CONFIG="${CMAKE_SOURCE_DIR}/configs/charger.json"
    )
    if(APPLE AND _USE_OBJCXX)
      set_source_files_properties(src/config_webui.cpp PROPERTIES LANGUAGE OBJCXX)
    endif()
  else()
    set(BUILD_CONFIG_WEB_UI OFF CACHE BOOL "Build the webview-powered config editor" FORCE)
  endif()
endif()

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

add_executable(can_protocol_vectors_tests
  tests/can_protocol_vectors_tests.cpp
  src/can_plc.cpp
)
target_include_directories(can_protocol_vectors_tests PRIVATE include libocpp/include)
target_link_libraries(can_protocol_vectors_tests PRIVATE
  everest::ocpp
  date::date-tz
  CURL::libcurl
  ${_JSON_SCHEMA_DEP}
  ${_EVEREST_SQLITE_DEP}
  Boost::log
  Boost::log_setup
  Boost::filesystem
  Boost::thread
  Boost::regex
  Boost::date_time
  Threads::Threads
)

add_executable(can_end_to_end_sim_tests
  tests/can_end_to_end_sim_tests.cpp
  src/can_plc.cpp
  Ref/Basic/src/firmware/util/crc8.cpp
)
target_include_directories(can_end_to_end_sim_tests PRIVATE include libocpp/include Ref/Basic/src/firmware/util)
target_link_libraries(can_end_to_end_sim_tests PRIVATE
  everest::ocpp
  date::date-tz
  CURL::libcurl
  ${_JSON_SCHEMA_DEP}
  ${_EVEREST_SQLITE_DEP}
  Boost::log
  Boost::log_setup
  Boost::filesystem
  Boost::thread
  Boost::regex
  Boost::date_time
  Threads::Threads
)

add_executable(auth_flow_tests
  tests/auth_flow_tests.cpp
  src/ocpp_adapter.cpp
  src/power_manager.cpp
  src/charger_config.cpp
  src/hardware_sim.cpp
  src/can_plc.cpp
  src/power_module_controller.cpp
)
target_include_directories(auth_flow_tests PRIVATE include libocpp/include)
target_link_libraries(auth_flow_tests PRIVATE
  everest::ocpp
  date::date-tz
  CURL::libcurl
  ${_JSON_SCHEMA_DEP}
  ${_EVEREST_SQLITE_DEP}
  Boost::log
  Boost::log_setup
  Boost::filesystem
  Boost::thread
  Boost::regex
  Boost::date_time
  Threads::Threads
)

add_executable(deterministic_vectors_tests
  tests/deterministic_vectors_tests.cpp
  src/ocpp_adapter.cpp
  src/power_manager.cpp
  src/charger_config.cpp
  src/hardware_sim.cpp
  src/can_plc.cpp
  src/power_module_controller.cpp
)
target_include_directories(deterministic_vectors_tests PRIVATE include libocpp/include)
target_link_libraries(deterministic_vectors_tests PRIVATE
  everest::ocpp
  date::date-tz
  CURL::libcurl
  ${_JSON_SCHEMA_DEP}
  ${_EVEREST_SQLITE_DEP}
  Boost::log
  Boost::log_setup
  Boost::filesystem
  Boost::thread
  Boost::regex
  Boost::date_time
  Threads::Threads
)

add_executable(pending_token_persistence_tests
  tests/pending_token_persistence_tests.cpp
  src/ocpp_adapter.cpp
  src/power_manager.cpp
  src/charger_config.cpp
  src/hardware_sim.cpp
  src/can_plc.cpp
  src/power_module_controller.cpp
)
target_include_directories(pending_token_persistence_tests PRIVATE include libocpp/include)
target_link_libraries(pending_token_persistence_tests PRIVATE
  everest::ocpp
  date::date-tz
  CURL::libcurl
  ${_JSON_SCHEMA_DEP}
  ${_EVEREST_SQLITE_DEP}
  Boost::log
  Boost::log_setup
  Boost::filesystem
  Boost::thread
  Boost::regex
  Boost::date_time
  Threads::Threads
)

add_executable(derate_fault_tests
  tests/derate_fault_tests.cpp
  src/ocpp_adapter.cpp
  src/power_manager.cpp
  src/charger_config.cpp
  src/hardware_sim.cpp
  src/can_plc.cpp
  src/power_module_controller.cpp
)
target_include_directories(derate_fault_tests PRIVATE include libocpp/include)
target_link_libraries(derate_fault_tests PRIVATE
  everest::ocpp
  date::date-tz
  CURL::libcurl
  ${_JSON_SCHEMA_DEP}
  ${_EVEREST_SQLITE_DEP}
  Boost::log
  Boost::log_setup
  Boost::filesystem
  Boost::thread
  Boost::regex
  Boost::date_time
  Threads::Threads
)

add_executable(limit_fallback_tests
  tests/limit_fallback_tests.cpp
  src/ocpp_adapter.cpp
  src/power_manager.cpp
  src/charger_config.cpp
  src/hardware_sim.cpp
  src/can_plc.cpp
  src/power_module_controller.cpp
)
target_include_directories(limit_fallback_tests PRIVATE include libocpp/include)
target_link_libraries(limit_fallback_tests PRIVATE
  everest::ocpp
  date::date-tz
  CURL::libcurl
  ${_JSON_SCHEMA_DEP}
  ${_EVEREST_SQLITE_DEP}
  Boost::log
  Boost::log_setup
  Boost::filesystem
  Boost::thread
  Boost::regex
  Boost::date_time
  Threads::Threads
)

add_executable(island_manager_tests
  tests/island_manager_tests.cpp
  src/island_manager.cpp
)
target_include_directories(island_manager_tests PRIVATE include)
target_link_libraries(island_manager_tests PRIVATE pthread)

add_executable(can_conformance_harness
  tests/can_conformance_harness.cpp
  src/charger_config.cpp
  src/can_plc.cpp
)
target_include_directories(can_conformance_harness PRIVATE include)
target_link_libraries(can_conformance_harness PRIVATE
  everest::ocpp
  date::date-tz
  CURL::libcurl
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
