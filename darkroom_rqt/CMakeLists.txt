cmake_minimum_required(VERSION 3.3)
project( darkroom_rqt )

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
#set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--export-all-symbols")

SET(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(qt_gui REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(roboy_middleware_msgs REQUIRED)
find_package(darkroom REQUIRED COMPONENTS TrackedObject Transform)
find_package(common_utilities REQUIRED COMPONENTS rviz_visualization UDPSocket)
find_package(ament_cmake_python REQUIRED)
#lint?


# Eigen
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
include_directories("${Eigen3_INCLUDE_DIRS}")

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

#catkin_python_setup()

#roslint_cpp()

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
    ${OpenCV_INCLUDE_DIRS}
    ../yamc/include
)

add_library( darkroom_rqt SHARED src/darkroom_rqt.cpp ${darkroom_rqt_MOCS} ${darkroom_rqt_UIS_H}
        qcustomplot/qcustomplot.cpp)
#add_dependencies(darkroom_rqt TrackedObject Transform)
target_link_libraries(darkroom_rqt ${darkroom_LIBRARIES}
       )
ament_target_dependencies( darkroom_rqt darkroom rclcpp roboy_middleware_msgs)

add_executable(test_this src/main.cpp ${test_RESOURCES_RCC})
#target_link_libraries(test_this ${catkin_LIBRARIES} )
ament_target_dependencies( test_this rclcpp)

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

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin/${PROJECT_NAME}
        INCLUDES DESTINATION include)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib)

install(
        DIRECTORY include/
        DESTINATION include
)

install(FILES
        package.xml
        DESTINATION share/${PROJECT_NAME}
        )

install(FILES plugin.xml
        DESTINATION share/${PROJECT_NAME}
        )

install(DIRECTORY resources
        DESTINATION share/${PROJECT_NAME}
        )

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

ament_export_dependencies(rqt_gui_cpp std_msgs
        roboy_middleware_msgs darkroom common_utilities)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} )
ament_package()