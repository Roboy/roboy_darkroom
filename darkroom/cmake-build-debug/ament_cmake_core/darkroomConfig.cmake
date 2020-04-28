# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_darkroom_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED darkroom_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(darkroom_FOUND FALSE)
  elseif(NOT darkroom_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(darkroom_FOUND FALSE)
  endif()
  return()
endif()
set(_darkroom_CONFIG_INCLUDED TRUE)

# output package information
if(NOT darkroom_FIND_QUIETLY)
  message(STATUS "Found darkroom: 0.0.0 (${darkroom_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'darkroom' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(darkroom_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${darkroom_DIR}/${_extra}")
endforeach()
