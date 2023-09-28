# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_robotvision_2023_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED robotvision_2023_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(robotvision_2023_FOUND FALSE)
  elseif(NOT robotvision_2023_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(robotvision_2023_FOUND FALSE)
  endif()
  return()
endif()
set(_robotvision_2023_CONFIG_INCLUDED TRUE)

# output package information
if(NOT robotvision_2023_FIND_QUIETLY)
  message(STATUS "Found robotvision_2023: 0.0.1 (${robotvision_2023_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'robotvision_2023' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${robotvision_2023_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(robotvision_2023_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${robotvision_2023_DIR}/${_extra}")
endforeach()
