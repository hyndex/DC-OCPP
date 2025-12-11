# Finder for everest-timer (header-only fallback).
include(FindPackageHandleStandardArgs)

set(_timer_hints)

find_path(EVEREST_TIMER_INCLUDE_DIR
          NAMES everest/timer.hpp
          HINTS ${_timer_hints}
          PATH_SUFFIXES include)

find_package_handle_standard_args(everest-timer DEFAULT_MSG EVEREST_TIMER_INCLUDE_DIR)

if(everest-timer_FOUND AND NOT TARGET everest::timer)
    add_library(everest::timer INTERFACE IMPORTED)
    set_target_properties(everest::timer PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EVEREST_TIMER_INCLUDE_DIR}")
endif()
