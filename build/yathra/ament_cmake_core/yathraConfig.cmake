# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_yathra_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED yathra_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(yathra_FOUND FALSE)
  elseif(NOT yathra_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(yathra_FOUND FALSE)
  endif()
  return()
endif()
set(_yathra_CONFIG_INCLUDED TRUE)

# output package information
if(NOT yathra_FIND_QUIETLY)
  message(STATUS "Found yathra: 0.0.0 (${yathra_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'yathra' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT yathra_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(yathra_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${yathra_DIR}/${_extra}")
endforeach()
