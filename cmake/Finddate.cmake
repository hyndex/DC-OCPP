# Minimal find module for Howard Hinnant date library with timezone support.

include(FindPackageHandleStandardArgs)

set(_date_hints)

find_path(DATE_INCLUDE_DIR
          NAMES date/date.h
          HINTS ${_date_hints}
          PATH_SUFFIXES include)

find_library(DATE_TZ_LIBRARY
             NAMES date-tz libdate-tz
             HINTS ${_date_hints}
             PATH_SUFFIXES lib)

find_package(Threads QUIET)
find_package_handle_standard_args(date DEFAULT_MSG DATE_INCLUDE_DIR)

if(date_FOUND)
    if(NOT TARGET date::date)
        add_library(date::date INTERFACE IMPORTED)
        set_target_properties(date::date PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${DATE_INCLUDE_DIR}")
    endif()

    if(DATE_TZ_LIBRARY AND NOT TARGET date::date-tz)
        add_library(date::date-tz UNKNOWN IMPORTED)
        set_target_properties(date::date-tz PROPERTIES
            IMPORTED_LOCATION "${DATE_TZ_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${DATE_INCLUDE_DIR}")
        if(Threads_FOUND)
            target_link_libraries(date::date-tz INTERFACE Threads::Threads)
        endif()
    endif()
endif()
