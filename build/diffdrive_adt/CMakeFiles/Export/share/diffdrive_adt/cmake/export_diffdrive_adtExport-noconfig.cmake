#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "diffdrive_adt::diffdrive_adt" for configuration ""
set_property(TARGET diffdrive_adt::diffdrive_adt APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(diffdrive_adt::diffdrive_adt PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdiffdrive_adt.so"
  IMPORTED_SONAME_NOCONFIG "libdiffdrive_adt.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS diffdrive_adt::diffdrive_adt )
list(APPEND _IMPORT_CHECK_FILES_FOR_diffdrive_adt::diffdrive_adt "${_IMPORT_PREFIX}/lib/libdiffdrive_adt.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
