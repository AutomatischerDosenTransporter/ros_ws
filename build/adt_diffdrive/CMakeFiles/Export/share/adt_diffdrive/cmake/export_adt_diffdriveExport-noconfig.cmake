#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "adt_diffdrive::adt_diffdrive" for configuration ""
set_property(TARGET adt_diffdrive::adt_diffdrive APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(adt_diffdrive::adt_diffdrive PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libadt_diffdrive.so"
  IMPORTED_SONAME_NOCONFIG "libadt_diffdrive.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS adt_diffdrive::adt_diffdrive )
list(APPEND _IMPORT_CHECK_FILES_FOR_adt_diffdrive::adt_diffdrive "${_IMPORT_PREFIX}/lib/libadt_diffdrive.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
