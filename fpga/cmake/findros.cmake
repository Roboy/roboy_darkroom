set(ROS_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/arm/opt/ros/kinetic)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/arm/usr/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/arm/usr/include/arm-linux-gnueabihf
        )
set(ros_LIBRARIES
        ${ROS_ROOT_DIR}/lib/librosconsole.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arm/usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
        ${ROS_ROOT_DIR}/lib/libroslz4.so
        ${ROS_ROOT_DIR}/lib/libroscpp.so
        ${ROS_ROOT_DIR}/lib/librostime.so
        ${ROS_ROOT_DIR}/lib/libroscpp_serialization.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arm/usr/lib/arm-linux-gnueabihf/libboost_system.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arm/usr/lib/arm-linux-gnueabihf/liblog4cxx.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arm/usr/lib/arm-linux-gnueabihf/libboost_regex.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arm/usr/lib/arm-linux-gnueabihf/libapr-1.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arm/usr/lib/arm-linux-gnueabihf/libaprutil-1.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arm/lib/arm-linux-gnueabihf/libuuid.so.1.3.0
        ${CMAKE_CURRENT_SOURCE_DIR}/arm/usr/lib/arm-linux-gnueabihf/libicuio.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arm/usr/lib/arm-linux-gnueabihf/libicudata.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arm/usr/lib/arm-linux-gnueabihf/libicule.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arm/usr/lib/arm-linux-gnueabihf/libicutu.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arm/usr/lib/arm-linux-gnueabihf/libicuuc.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arm/usr/lib/arm-linux-gnueabihf/libicui18n.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arm/lib/arm-linux-gnueabihf/libdl.so.2
        ${CMAKE_CURRENT_SOURCE_DIR}/arm/usr/lib/arm-linux-gnueabihf/libtinyxml.so
        )
message(STATUS "ros_LIBRARIES: ${ros_LIBRARIES}" )