# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_shelfbot_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED shelfbot_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(shelfbot_FOUND FALSE)
  elseif(NOT shelfbot_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(shelfbot_FOUND FALSE)
  endif()
  return()
endif()
set(_shelfbot_CONFIG_INCLUDED TRUE)

# output package information
if(NOT shelfbot_FIND_QUIETLY)
  message(STATUS "Found shelfbot: 0.0.1 (${shelfbot_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'shelfbot' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${shelfbot_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(shelfbot_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${shelfbot_DIR}/${_extra}")
endforeach()
