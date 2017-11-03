cmake_minimum_required(VERSION 2.8.3)
project(darkroom)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation tf tf_conversions visualization_msgs
		roboy_communication_middleware common_utilities )

# Eigen
find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

# yaml-cpp
pkg_check_modules( yaml-cpp REQUIRED yaml-cpp)
include_directories(${yaml-cpp_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV 3.1.0 COMPONENTS opencv_core opencv_highgui opencv_calib3d REQUIRED)

catkin_package(CATKIN_DEPENDS message_runtime roboy_communication_middleware common_utilities INCLUDE_DIRS include
        LIBRARIES ParticleFilter PoseEstimatorSensorCloud PoseEstimatorSensorDistance PoseEstimatorSensorDistances
        Transform Triangulate LighthouseEstimator LighthouseSimulator Sensor TrackedObject )

include_directories(
		include/
        include/darkroom/mavmap/src
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${ncurses_INCLUDE_DIRS}
)
#==============================================================================
# Application
#==============================================================================

add_library( ParticleFilter src/ParticleFilter.cpp )

add_library( PoseEstimatorSensorCloud src/PoseEstimatorSensorCloud.cpp )

add_library( PoseEstimatorSensorDistance src/PoseEstimatorSensorDistance.cpp )

add_library( PoseEstimatorSensorDistances src/PoseEstimatorSensorDistances.cpp )

add_library( Transform src/Transform.cpp )
target_link_libraries( Transform ${catkin_LIBRARIES} )

add_library( Triangulate src/Triangulation.cpp )
target_link_libraries( Triangulate ${catkin_LIBRARIES} )

add_library( LighthouseSimulator src/LighthouseSimulator.cpp )
add_dependencies(LighthouseSimulator roboy_communication_middleware_gencpp )
target_link_libraries( LighthouseSimulator ${catkin_LIBRARIES} Transform)

add_library( LighthouseEstimator src/LighthouseEstimator.cpp  include/darkroom/mavmap/src/base3d/projection.cc
        include/darkroom/epnp/epnp.cpp include/darkroom/mavmap/src/base3d/p3p.cc include/darkroom/mavmap/src/util/math.cc)
add_dependencies(LighthouseEstimator roboy_communication_middleware_gencpp )
target_link_libraries( LighthouseEstimator ${catkin_LIBRARIES} PoseEstimatorSensorCloud PoseEstimatorSensorDistance
		PoseEstimatorSensorDistances Transform Triangulate )

add_library( Sensor src/Sensor.cpp )
target_link_libraries( Sensor ${catkin_LIBRARIES} )

add_library( TrackedObject src/TrackedObject.cpp )
add_dependencies(TrackedObject roboy_communication_middleware_gencpp )
target_link_libraries( TrackedObject ${yaml-cpp_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS}
		Sensor LighthouseEstimator )

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        ParticleFilter
		PoseEstimatorSensorCloud
        PoseEstimatorSensorDistance
        PoseEstimatorSensorDistances
        Transform
        Triangulate
        LighthouseEstimator
        Sensor
        TrackedObject
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
