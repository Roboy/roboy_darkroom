cmake_minimum_required(VERSION 2.8.3)
project( darkroom_rqt )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")

find_package(catkin REQUIRED COMPONENTS roscpp roslint rqt_gui rqt_gui_cpp std_msgs
        roboy_middleware_msgs darkroom common_utilities )

# Eigen
find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

# alglib
find_package(ALGLIB REQUIRED)
include_directories(${ALGLIB_INCLUDE_DIRS})

include_directories( include qcustomplot)

## System dependencies are found with CMake's conventions

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    find_package(Qt5 COMPONENTS Core Widgets Qml Quick REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

roslint_cpp()

catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs
        roboy_middleware_msgs darkroom common_utilities )

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(darkroom_rqt_MOCS include/darkroom_rqt/darkroom_rqt.hpp qcustomplot/qcustomplot.h)
  qt5_wrap_ui(darkroom_rqt_UIS_H src/darkroom_rqt.ui)
  qt5_add_resources(test_RESOURCES_RCC darkroom_rqt.qrc)
else()
  qt4_wrap_cpp(darkroom_rqt_MOCS include/darkroom_rqt/darkroom_rqt.hpp qcustomplot/qcustomplot.h)
  qt4_wrap_ui(darkroom_rqt_UIS_H src/darkroom_rqt.ui)
endif()

# OpenCV
find_package(OpenCV 3.1.0 COMPONENTS opencv_core opencv_highgui opencv_calib3d REQUIRED)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}/..
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ../yamc/include
)

add_library( darkroom_rqt src/darkroom_rqt.cpp ${darkroom_rqt_MOCS} ${darkroom_rqt_UIS_H}
        qcustomplot/qcustomplot.cpp)
add_dependencies(darkroom_rqt roboy_middleware_msgs_gencpp)
target_link_libraries(darkroom_rqt ${catkin_LIBRARIES} )

add_executable(test_this src/main.cpp ${test_RESOURCES_RCC})
target_link_libraries(test_this ${catkin_LIBRARIES} )

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(darkroom_rqt Qt5::Widgets )
  target_link_libraries(test_this Qt5::Widgets Qt5::Qml Qt5::Quick)
else()
  target_link_libraries(darkroom_rqt ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
  target_link_libraries(test_this ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

#############
## Install ##
#############

install(TARGETS darkroom_rqt
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )