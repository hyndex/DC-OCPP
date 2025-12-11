# Finder for everest-log static library.
include(FindPackageHandleStandardArgs)

set(_log_hints)

find_library(EVEREST_LOG_LIBRARY
             NAMES everest_log libeverest_log
             HINTS ${_log_hints}
             PATH_SUFFIXES lib)

find_path(EVEREST_LOG_INCLUDE_DIR
          NAMES everest/logging.hpp
          HINTS ${_log_hints}
          PATH_SUFFIXES include)

find_package_handle_standard_args(everest-log DEFAULT_MSG EVEREST_LOG_LIBRARY EVEREST_LOG_INCLUDE_DIR)

if(everest-log_FOUND AND NOT TARGET everest::log)
    add_library(everest::log UNKNOWN IMPORTED)
    set_target_properties(everest::log PROPERTIES
        IMPORTED_LOCATION "${EVEREST_LOG_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${EVEREST_LOG_INCLUDE_DIR}")
endif()
