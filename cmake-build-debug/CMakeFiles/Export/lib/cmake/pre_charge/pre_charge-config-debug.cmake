#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pre_charge::pre_charge" for configuration "Debug"
set_property(TARGET pre_charge::pre_charge APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(pre_charge::pre_charge PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libpre_charged.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS pre_charge::pre_charge )
list(APPEND _IMPORT_CHECK_FILES_FOR_pre_charge::pre_charge "${_IMPORT_PREFIX}/lib/libpre_charged.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
