# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rectagletest_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rectagletest_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rectagletest_FOUND FALSE)
  elseif(NOT rectagletest_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rectagletest_FOUND FALSE)
  endif()
  return()
endif()
set(_rectagletest_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rectagletest_FIND_QUIETLY)
  message(STATUS "Found rectagletest: 0.0.1 (${rectagletest_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rectagletest' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rectagletest_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rectagletest_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rectagletest_DIR}/${_extra}")
endforeach()
