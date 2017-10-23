#include "darkroom_fpga/DarkRoom.hpp"
#include <iostream>

DarkRoom::DarkRoom(void *h2p_lw_darkroom_addr) : h2p_lw_darkroom_addr(h2p_lw_darkroom_addr) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "darkroom_fpga", ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    sensor_pub = nh->advertise<roboy_communication_middleware::DarkRoom>("/roboy/middleware/DarkRoom/sensors", 10);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    getData = true;

    sensor_thread = boost::shared_ptr<thread>(new thread(&DarkRoom::getSensorValues, this));
    sensor_thread->detach();
}

DarkRoom::~DarkRoom() {
    getData = false;
    if (sensor_thread != nullptr) {
        if (sensor_thread->joinable()) {
            ROS_INFO("Waiting for sensor thread to terminate");
            sensor_thread->join();
        }
    }
};


/**
    Poll sensor measurements 240 times per second and send the results via ROS.
    The ROS message will contain an array of 32-bit values. Format:
        bit  31      lighthouse_id
        bit  30      axis
        bit  29      valid
        bits 28:0    duration (divide by 50 to get microseconds)    
*/
void DarkRoom::getSensorValues() {

    // number of sensors to read
    int NUM_SENSORS = 32;


    ros::Rate rate(240); // two times faster than actually needed

    while (ros::ok() && getData) {

        roboy_communication_middleware::DarkRoom msg;
        bool any_valid = true;
        // read all sensors
        for (int i = 0; i < NUM_SENSORS; i++) {
            /*
            combined data for each sensor is available on addresses 0 to (NUM_SENSORS - 1)
            layout:
                bit  31      lighthouse_id
                bit  30      axis
                bit  29      valid
                bits 28:0    duration (divide by 50 to get microseconds)    
            */

            // ROS
            int combined_data = IORD(h2p_lw_darkroom_addr, i);
            msg.sensor_value.push_back(combined_data);

            // CONSOLE
            int nskip_to_sweep = combined_data & 0x1FFFFFFF;
            int valid = (combined_data & 0x20000000) >> 29;
            int current_axis = (combined_data & 0x40000000) >> 30;
            int lighthouse_id = (combined_data & 0x80000000) >> 31;
            if (valid) {
                ROS_INFO_STREAM_THROTTLE(1, "sensor(" << i << "): id=" << lighthouse_id << "\taxis=" << current_axis
                                                      << "\tduration=" << nskip_to_sweep);
            }else{
                any_valid = false;
            }
        }

        if(any_valid){
            ROS_INFO_THROTTLE(5, "i can see the light");
        }else{
            ROS_INFO_THROTTLE(5, "some sensors are not visible to a lighthouse");
        }

        sensor_pub.publish(msg);
        rate.sleep();

    }

}
