# - Find ALGLIB
# Find the native ALGLIB includes and library
#
#  ALGLIB_INCLUDE_DIRS    - where to find alglib
#  ALGLIB_LIBRARIES   - List of libraries when using alglib.
#  ALGLIB_FOUND       - True if alglib found.

include(FindPkgConfig)
include(FindPackageHandleStandardArgs)

# Use pkg-config to get hints about paths
#pkg_check_modules(ALGLIB_PKGCONF REQUIRED alglib)

# Include dir
find_path(ALGLIB_INCLUDE_DIRS
  NAMES alglibinternal.h
    alglibmisc.h
    ap.h
    dataanalysis.h
    diffequations.h
    fasttransforms.h
    integration.h
    interpolation.h
    linalg.h
    optimization.h
    solvers.h
    specialfunctions.h
    statistics.h
    stdafx.h
  PATHS /usr/include/libalglib/
)

# Finally the library itself
find_library(ALGLIB_LIBRARIES
  NAMES alglib
  PATHS /usr/lib/x86_64-linux-gnu/
)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(ALGLIB DEFAULT_MSG ALGLIB_LIBRARIES ALGLIB_INCLUDE_DIRS)

mark_as_advanced (ALGLIB_LIBRARIES ALGLIB_INCLUDES)
if(ALGLIB_PKGCONF_FOUND)
  set(ALGLIB_LIBRARIES ${ALGLIB_LIBRARY} ${ALGLIB_PKGCONF_LIBRARIES})
  set(ALGLIB_INCLUDE_DIRS ${ALGLIB_INCLUDE_DIR} ${ALGLIB_PKGCONF_INCLUDE_DIRS})
  set(ALGLIB_FOUND yes)
else()
  set(ALGLIB_LIBRARIES)
  set(ALGLIB_INCLUDE_DIRS)
  set(ALGLIB_FOUND no)
endif()