#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hand_control_core::core" for configuration ""
set_property(TARGET hand_control_core::core APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(hand_control_core::core PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libhand_control_core.a"
  )

list(APPEND _cmake_import_check_targets hand_control_core::core )
list(APPEND _cmake_import_check_files_for_hand_control_core::core "${_IMPORT_PREFIX}/lib/libhand_control_core.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
