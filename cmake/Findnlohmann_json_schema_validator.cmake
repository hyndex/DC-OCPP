# Finder for nlohmann_json_schema_validator built artifacts (static lib + headers).
include(FindPackageHandleStandardArgs)

set(_njsv_hints)

find_library(NLOHMANN_JSON_SCHEMA_VALIDATOR_LIBRARY
             NAMES nlohmann_json_schema_validator libnlohmann_json_schema_validator
             HINTS ${_njsv_hints}
             PATH_SUFFIXES lib)

find_path(NLOHMANN_JSON_SCHEMA_VALIDATOR_INCLUDE_DIR
          NAMES nlohmann/json-schema.hpp
          HINTS ${_njsv_hints}
          PATH_SUFFIXES src include)

find_package_handle_standard_args(nlohmann_json_schema_validator DEFAULT_MSG
                                  NLOHMANN_JSON_SCHEMA_VALIDATOR_LIBRARY
                                  NLOHMANN_JSON_SCHEMA_VALIDATOR_INCLUDE_DIR)

if(nlohmann_json_schema_validator_FOUND AND NOT TARGET nlohmann_json_schema_validator)
    add_library(nlohmann_json_schema_validator UNKNOWN IMPORTED)
    set_target_properties(nlohmann_json_schema_validator PROPERTIES
        IMPORTED_LOCATION "${NLOHMANN_JSON_SCHEMA_VALIDATOR_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${NLOHMANN_JSON_SCHEMA_VALIDATOR_INCLUDE_DIR}")
    if(TARGET nlohmann_json::nlohmann_json)
        target_link_libraries(nlohmann_json_schema_validator INTERFACE nlohmann_json::nlohmann_json)
    endif()
endif()
