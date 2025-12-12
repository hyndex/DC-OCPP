# Finder for everest-log static library.
include(FindPackageHandleStandardArgs)

# If the target already exists (e.g., provided via FetchContent), reuse it directly.
if(TARGET everest_log OR TARGET everest-log)
    set(_log_target everest_log)
    if(TARGET everest-log)
        set(_log_target everest-log)
    endif()
    if(NOT EVEREST_LOG_INCLUDE_DIR)
        get_target_property(_log_includes ${_log_target} INTERFACE_INCLUDE_DIRECTORIES)
        if(_log_includes)
            list(GET _log_includes 0 EVEREST_LOG_INCLUDE_DIR)
        endif()
    endif()
    set(EVEREST_LOG_LIBRARY ${_log_target})
    set(everest-log_FOUND TRUE)
    if(NOT TARGET everest::log)
        add_library(everest::log ALIAS ${_log_target})
    endif()
    return()
endif()

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
