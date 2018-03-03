#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <iostream>
#include <darkroom/Transform.hpp>
#include <ros/ros.h>

#define MAX_LEN 10000
struct region {        /* Defines "structure" of shared memory */
    int len;
    char buf[MAX_LEN];
};
struct region *rptr;
using namespace std;

int fd;

int main(){

    fd =  shm_open("/openVRPose", O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
    if (ftruncate(fd, sizeof(struct region)) == -1)
        cout << "ftruncate failed" << endl;


    /* Map shared memory object */
    rptr = (region*)mmap(NULL, sizeof(struct region),
                         PROT_READ , MAP_SHARED, fd, 0);
    if (rptr == MAP_FAILED)
        cout << "mmap failed" << endl;

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "openvr_pose_publisher",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle());

    DarkRoom::Transform trans;

    Matrix4d controller_pose, lighthouse_pose[2];
    lighthouse_pose[0].setIdentity();
    lighthouse_pose[1].setIdentity();
    while(ros::ok()){
        int n = 0;
        float val;
        // controller
        for(int i=0;i<4;i++) {
            for (int j = 0; j < 4; j++) {
                memcpy(&val,&rptr->buf[n*4],sizeof(float));
                controller_pose(j,i) = val;
                n++;
            }
        }
        ROS_INFO_STREAM_THROTTLE(1, "controller_pose:\n"<< controller_pose);
        // lighthouse 0
        for(int i=0;i<3;i++) {
            for (int j = 0; j < 4; j++) {
                memcpy(&val,&rptr->buf[n*4],sizeof(float));
                lighthouse_pose[0](i,j) = val;
                n++;
            }
        }
        ROS_INFO_STREAM_THROTTLE(1, "lighthouse 0:\n"<< lighthouse_pose[0]);
        // lighthouse 0
        for(int i=0;i<3;i++) {
            for (int j = 0; j < 4; j++) {
                memcpy(&val,&rptr->buf[n*4],sizeof(float));
                lighthouse_pose[1](i,j) = val;
                n++;
            }
        }
        ROS_INFO_STREAM_THROTTLE(1, "lighthouse 1:\n" << lighthouse_pose[1]);
        tf::Transform frame[3];
        trans.getTFtransform(controller_pose,frame[0]);
        trans.getTFtransform(lighthouse_pose[0],frame[1]);
        trans.getTFtransform(lighthouse_pose[1],frame[2]);
        trans.publishTF(frame[0],"world","controller");
        trans.publishTF(frame[1],"world","lighthouse0");
        trans.publishTF(frame[2],"world","lighthouse1");
    }
}
