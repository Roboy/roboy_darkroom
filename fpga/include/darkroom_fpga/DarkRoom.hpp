#pragma once

#include <ros/ros.h>
#include <roboy_communication_middleware/DarkRoom.h>
#include <thread>

#define IORD(base,reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base,reg,data) (*(((volatile int32_t*)base)+reg)=data)
#define IORD_8DIRECT(base, offset) (*(((volatile int8_t*)base)+offset))
#define IORD_16DIRECT(base, offset) (*(((volatile int16_t*)base)+(offset>>1)))
#define IORD_32DIRECT(base, offset) (*(((volatile int32_t*)base)+(offset>>2)))
#define IOWR_8DIRECT(base, offset, data) (*(((volatile int8_t*)base)+offset)=data)
#define IOWR_16DIRECT(base, offset, data) (*(((volatile int16_t*)base)+(offset>>1))=data)
#define IOWR_32DIRECT(base, offset, data) (*(((volatile int32_t*)base)+(offset>>2))=data)

using namespace std;

class DarkRoom{
public:
    DarkRoom(void* h2p_lw_darkroom_addr);
    ~DarkRoom();
    void getSensorValues();
private:
    void *h2p_lw_darkroom_addr;
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Publisher sensor_pub;
    bool getData = true;
    boost::shared_ptr<thread> sensor_thread = nullptr;
};
