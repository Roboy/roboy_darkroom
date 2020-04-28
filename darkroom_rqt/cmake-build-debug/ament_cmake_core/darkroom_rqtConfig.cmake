# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_darkroom_rqt_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED darkroom_rqt_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(darkroom_rqt_FOUND FALSE)
  elseif(NOT darkroom_rqt_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(darkroom_rqt_FOUND FALSE)
  endif()
  return()
endif()
set(_darkroom_rqt_CONFIG_INCLUDED TRUE)

# output package information
if(NOT darkroom_rqt_FIND_QUIETLY)
  message(STATUS "Found darkroom_rqt: 0.0.0 (${darkroom_rqt_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'darkroom_rqt' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(darkroom_rqt_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${darkroom_rqt_DIR}/${_extra}")
endforeach()
