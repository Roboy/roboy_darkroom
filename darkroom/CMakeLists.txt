cmake_minimum_required(VERSION 2.8.3)
project(darkroom)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation tf tf_conversions visualization_msgs
		roboy_communication_middleware common_utilities robot_localization xmlrpcpp pcl_ros epnp)

# Eigen
find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

# yaml-cpp
pkg_check_modules( yaml-cpp REQUIRED yaml-cpp)
include_directories(${yaml-cpp_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV 3.1.0 COMPONENTS opencv_core opencv_highgui opencv_calib3d REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# alglib
find_package(ALGLIB REQUIRED)
include_directories(${ALGLIB_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS message_runtime roboy_communication_middleware common_utilities robot_localization xmlrpcpp
		pcl_ros epnp
		INCLUDE_DIRS include
        LIBRARIES PoseEstimatorSensorCloud Transform Triangulate LighthouseEstimator LighthouseSimulator
        Sensor TrackedObject )

include_directories(
		include/
        ${catkin_INCLUDE_DIRS}
        ${ncurses_INCLUDE_DIRS}
)
#==============================================================================
# Application
#==============================================================================

add_library( PoseEstimatorSensorCloud src/PoseEstimatorSensorCloud.cpp )
add_library( InYourGibbousPhase src/InYourGibbousPhase.cpp )
add_library( InYourGibbousPhase3 src/InYourGibbousPhase3.cpp )

add_executable(testMe src/testMe.cpp)
target_link_libraries(testMe ${ALGLIB_LIBRARIES})

add_library( Sensor src/Sensor.cpp )
target_link_libraries( Sensor ${catkin_LIBRARIES} )

add_library( Transform src/Transform.cpp )
target_link_libraries( Transform ${catkin_LIBRARIES} )

add_library( Triangulate src/Triangulation.cpp )
target_link_libraries( Triangulate ${catkin_LIBRARIES} )

add_library( Utilities src/Utilities.cpp )
target_link_libraries( Utilities ${catkin_LIBRARIES} )

add_library( LighthouseSimulator src/LighthouseSimulator.cpp )
add_dependencies(LighthouseSimulator roboy_communication_middleware_gencpp )
target_link_libraries( LighthouseSimulator ${catkin_LIBRARIES} Transform)

add_library( LighthouseEstimator src/LighthouseEstimator.cpp )
add_dependencies(LighthouseEstimator roboy_communication_middleware_gencpp )
target_link_libraries( LighthouseEstimator ${catkin_LIBRARIES} PoseEstimatorSensorCloud Transform Triangulate
		InYourGibbousPhase InYourGibbousPhase3 ${ALGLIB_LIBRARIES})

add_library( TrackedObject src/TrackedObject.cpp )
add_dependencies(TrackedObject roboy_communication_middleware_gencpp )
target_link_libraries( TrackedObject ${yaml-cpp_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} Sensor LighthouseEstimator Utilities )

add_library( DarkRoomVisualPlugin src/gazebo/DarkRoomVisualPlugin.cpp )
add_dependencies(DarkRoomVisualPlugin roboy_communication_middleware_gencpp )
target_link_libraries( DarkRoomVisualPlugin ${catkin_LIBRARIES} )

add_library( LighthouseVisualPlugin src/gazebo/LighthouseVisualPlugin.cpp )
add_dependencies(LighthouseVisualPlugin roboy_communication_middleware_gencpp )
target_link_libraries( LighthouseVisualPlugin ${catkin_LIBRARIES} Transform )

add_library( ImuModelPlugin src/gazebo/ImuModelPlugin.cpp )
target_link_libraries( ImuModelPlugin ${catkin_LIBRARIES} )

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
		PoseEstimatorSensorCloud
        Transform
        Triangulate
        LighthouseEstimator
        Sensor
        TrackedObject
        ImuModelPlugin
	    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/darkroom/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        )

install(DIRECTORY include/darkroom/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
