cmake_minimum_required(VERSION 3.3)
project(darkroom)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -fpic")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")
SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
#find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
#find_package(tf_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(roboy_middleware_msgs REQUIRED)
find_package(common_utilities REQUIRED COMPONENTS rviz_visualization UDPSocket)
find_package(robot_localization REQUIRED COMPONENTS rl_lib)
find_package(xmlrpcpp REQUIRED)
#find_package(pcl_ros REQUIRED)
find_package(epnp REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(rcl_yaml_param_parser REQUIRED)
		#
#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation tf tf_conversions visualization_msgs
#		roboy_middleware_msgs common_utilities robot_localization xmlrpcpp pcl_ros epnp)

# Eigen
#find_package(PkgConfig REQUIRED )
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
include_directories("${Eigen3_INCLUDE_DIRS}")

# yaml-cpp
find_package(yaml_cpp_vendor REQUIRED)
find_package( yaml-cpp REQUIRED yaml-cpp)
include_directories(${yaml-cpp_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV 3.1.0 COMPONENTS opencv_core opencv_highgui opencv_calib3d REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# alglib
find_package(ALGLIB REQUIRED)
include_directories(${ALGLIB_INCLUDE_DIRS})



include_directories(
		include/

        ${ncurses_INCLUDE_DIRS}
        ../yamc/include
)
#==============================================================================
# Application
#==============================================================================
add_library( Sensor src/Sensor.cpp )
#target_link_libraries( Sensor  )
ament_target_dependencies(Sensor rclcpp)

add_library( PoseEstimatorSensorCloud src/PoseEstimatorSensorCloud.cpp )
add_library( InYourGibbousPhase src/InYourGibbousPhase.cpp )
add_library( InYourGibbousPhase3 src/InYourGibbousPhase3.cpp )
add_library( InYourGibbousPhase4 src/InYourGibbousPhase4.cpp )
add_library( PoseEstimatorMultiLighthouse src/PoseEstimatorMultiLighthouse.cpp )

#add_executable(testMe src/testMe.cpp)
#target_link_libraries(testMe ${ALGLIB_LIBRARIES})
#ament_target_dependencies(testMe rclcpp)



add_library( Transform SHARED src/Transform.cpp )
ament_target_dependencies( Transform  rclcpp)

add_library( Triangulate src/Triangulation.cpp )
ament_target_dependencies( Triangulate rclcpp )

add_library( Utilities src/Utilities.cpp )
target_link_libraries(Utilities   ${rcl_yaml_param_parser_LIBRARIES})
ament_target_dependencies( Utilities rclcpp yaml_cpp_vendor)

#add_library( LighthouseSimulator src/LighthouseSimulator.cpp )
##add_dependencies(LighthouseSimulator roboy_middleware_msgs_gencpp )
#target_link_libraries( LighthouseSimulator  Transform)
#ament_target_dependencies( LighthouseSimulator rclcpp roboy_middleware_msgs)
#
add_library( LighthouseEstimator src/LighthouseEstimator.cpp )
#add_dependencies(LighthouseEstimator roboy_middleware_msgs_gencpp )
target_link_libraries( LighthouseEstimator  PoseEstimatorSensorCloud Transform Triangulate
		InYourGibbousPhase InYourGibbousPhase3 InYourGibbousPhase4 PoseEstimatorMultiLighthouse ${ALGLIB_LIBRARIES} ${epnp_LIBRARIES} ${common_utilities_LIBRARIES})
ament_target_dependencies( LighthouseEstimator rclcpp yaml_cpp_vendor roboy_middleware_msgs epnp common_utilities)

add_library( TrackedObject SHARED src/TrackedObject.cpp )
#add_dependencies(TrackedObject rl_lib )
target_link_libraries( TrackedObject ${Boost_LIBRARIES} ${OpenCV_LIBS} ${common_utilities_LIBRARIES} Sensor LighthouseEstimator Utilities ${robot_localization_LIBRARIES} ) #rl_lib rviz_visualization UDPSocket
ament_target_dependencies( TrackedObject rclcpp yaml_cpp_vendor robot_localization common_utilities)

#add_executable(publishOpenVRPoses src/publishOpenVRPoses.cpp)
#target_link_libraries( publishOpenVRPoses  Transform -lrt)
#ament_target_dependencies(publishOpenVRPoses rclcpp)

ament_export_dependencies(rclcpp yaml_cpp_vendor roboy_middleware_msgs common_utilities robot_localization xmlrpcpp
		epnp)
ament_export_include_directories(include)
ament_export_libraries(PoseEstimatorSensorCloud Transform Triangulate LighthouseEstimator
		Sensor TrackedObject )
ament_package()

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
		Sensor
		TrackedObject
		PoseEstimatorSensorCloud
        Transform
        Triangulate
        LighthouseEstimator
##        ImuModelPlugin
	    LIBRARY DESTINATION lib/${PROJECT_NAME}
        )

install(TARGETS
		Sensor
		TrackedObject
		PoseEstimatorSensorCloud
		Transform
		Triangulate
		LighthouseEstimator
		##        ImuModelPlugin
		DESTINATION lib
		)

install(FILES
        package.xml
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/darkroom/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.hpp"
        )

install(DIRECTORY include/darkroom/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        )
