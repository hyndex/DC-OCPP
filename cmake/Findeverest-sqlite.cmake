# Finder for everest-sqlite static library.
include(FindPackageHandleStandardArgs)

set(_sqlite_hints)

find_library(EVEREST_SQLITE_LIBRARY
             NAMES everest_sqlite libeverest_sqlite
             HINTS ${_sqlite_hints}
             PATH_SUFFIXES lib)

find_path(EVEREST_SQLITE_INCLUDE_DIR
          NAMES everest/database/sqlite/connection.hpp
          HINTS ${_sqlite_hints}
          PATH_SUFFIXES include)

find_package_handle_standard_args(everest-sqlite DEFAULT_MSG EVEREST_SQLITE_LIBRARY EVEREST_SQLITE_INCLUDE_DIR)

if(everest-sqlite_FOUND AND NOT TARGET everest::sqlite)
    add_library(everest::sqlite UNKNOWN IMPORTED)
    set_target_properties(everest::sqlite PROPERTIES
        IMPORTED_LOCATION "${EVEREST_SQLITE_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${EVEREST_SQLITE_INCLUDE_DIR}")
endif()
