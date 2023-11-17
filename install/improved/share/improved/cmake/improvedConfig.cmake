# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_improved_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED improved_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(improved_FOUND FALSE)
  elseif(NOT improved_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(improved_FOUND FALSE)
  endif()
  return()
endif()
set(_improved_CONFIG_INCLUDED TRUE)

# output package information
if(NOT improved_FIND_QUIETLY)
  message(STATUS "Found improved: 0.0.1 (${improved_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'improved' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${improved_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(improved_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${improved_DIR}/${_extra}")
endforeach()
