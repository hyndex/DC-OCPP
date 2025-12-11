# Header-only nlohmann::json fallback finder.
include(FindPackageHandleStandardArgs)

set(_nlohmann_hints)

find_path(NLOHMANN_JSON_INCLUDE_DIR
          NAMES nlohmann/json.hpp
          HINTS ${_nlohmann_hints}
          PATH_SUFFIXES nlohmann single_include)

find_package_handle_standard_args(nlohmann_json DEFAULT_MSG NLOHMANN_JSON_INCLUDE_DIR)

if(nlohmann_json_FOUND AND NOT TARGET nlohmann_json::nlohmann_json)
    add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED)
    set_target_properties(nlohmann_json::nlohmann_json PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${NLOHMANN_JSON_INCLUDE_DIR}")
endif()
