# Finder for everest-evse_security static library.
include(FindPackageHandleStandardArgs)

set(_evse_hints)

find_library(EVSE_SECURITY_LIBRARY
             NAMES evse_security libevse_security
             HINTS ${_evse_hints}
             PATH_SUFFIXES lib lib/evse_security)

find_path(EVSE_SECURITY_INCLUDE_DIR
          NAMES evse_security/evse_security.hpp
          HINTS ${_evse_hints}
          PATH_SUFFIXES include)

find_package(OpenSSL REQUIRED)
find_package_handle_standard_args(everest-evse_security DEFAULT_MSG EVSE_SECURITY_LIBRARY EVSE_SECURITY_INCLUDE_DIR)

if(everest-evse_security_FOUND AND NOT TARGET everest::evse_security)
    add_library(everest::evse_security UNKNOWN IMPORTED)
    set_target_properties(everest::evse_security PROPERTIES
        IMPORTED_LOCATION "${EVSE_SECURITY_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${EVSE_SECURITY_INCLUDE_DIR}")
    target_link_libraries(everest::evse_security INTERFACE OpenSSL::SSL OpenSSL::Crypto)
endif()
