# Minimal placeholder finder for fsm dependency used by libocpp builds.
include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(fsm DEFAULT_MSG)

if(NOT TARGET fsm)
    add_library(fsm INTERFACE IMPORTED)
endif()

set(fsm_FOUND TRUE)
