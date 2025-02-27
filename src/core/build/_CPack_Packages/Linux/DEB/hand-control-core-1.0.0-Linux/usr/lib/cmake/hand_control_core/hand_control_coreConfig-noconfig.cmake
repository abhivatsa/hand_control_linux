#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hand_control::math_lib" for configuration ""
set_property(TARGET hand_control::math_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(hand_control::math_lib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmath_lib.a"
  )

list(APPEND _cmake_import_check_targets hand_control::math_lib )
list(APPEND _cmake_import_check_files_for_hand_control::math_lib "${_IMPORT_PREFIX}/lib/libmath_lib.a" )

# Import target "hand_control::robotics_lib_haptic_device" for configuration ""
set_property(TARGET hand_control::robotics_lib_haptic_device APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(hand_control::robotics_lib_haptic_device PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librobotics_lib_haptic_device.a"
  )

list(APPEND _cmake_import_check_targets hand_control::robotics_lib_haptic_device )
list(APPEND _cmake_import_check_files_for_hand_control::robotics_lib_haptic_device "${_IMPORT_PREFIX}/lib/librobotics_lib_haptic_device.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
