# Finder for libwebsockets (shared).
include(FindPackageHandleStandardArgs)

set(_lws_hints)

find_library(LIBWEBSOCKETS_LIBRARY
             NAMES websockets_shared websockets libwebsockets
             HINTS ${_lws_hints}
             PATH_SUFFIXES lib)

find_path(LIBWEBSOCKETS_INCLUDE_DIR
          NAMES libwebsockets.h
          HINTS ${_lws_hints}
          PATH_SUFFIXES include include/libwebsockets)

find_package_handle_standard_args(libwebsockets DEFAULT_MSG LIBWEBSOCKETS_LIBRARY LIBWEBSOCKETS_INCLUDE_DIR)

if(libwebsockets_FOUND)
    if(NOT TARGET websockets_shared)
        add_library(websockets_shared UNKNOWN IMPORTED)
        set_target_properties(websockets_shared PROPERTIES
            IMPORTED_LOCATION "${LIBWEBSOCKETS_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${LIBWEBSOCKETS_INCLUDE_DIR}")
    endif()
    if(NOT TARGET Libwebsockets::Libwebsockets)
        add_library(Libwebsockets::Libwebsockets ALIAS websockets_shared)
    endif()
endif()
