#include "darkroom/LighthouseSimulator.hpp"

int LighthouseSimulator::class_counter = 0;

LighthouseSimulator::LighthouseSimulator(int id):id(id){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "LighthouseSimulator",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    sensors_pub = nh->advertise<roboy_communication_middleware::DarkRoom>(
            "/roboy/middleware/DarkRoom/sensors", 1);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    sensor_position[0] << 0,    0,      0,      1;
    sensor_position[1] << 0,    0,      0.2,    1;
    sensor_position[2] << 0,    0.2,    0,      1;
    sensor_position[3] << 0,    0.2,    0.2,    1;
    sensor_position[4] << 0.2,  0,      0,      1;
    sensor_position[5] << 0.2,  0,      0.2,    1;
    sensor_position[6] << 0.2,  0.2,    0,      1;
    sensor_position[7] << 0.2,  0.2,    0.2,    1;

    class_counter++;
}

LighthouseSimulator::~LighthouseSimulator(){
    {
        lock_guard<mutex> lock(mux);
        sensor_publishing = false;
    }
}

void LighthouseSimulator::PublishSensorData(){
    ros::Rate rate(120);
    bool angle_switch = false;
    while(sensor_publishing){
        Matrix4d RT_object2lighthouse;
        if(!getTransform((id==0?"simulated_object_lighthouse1":"simulated_object_lighthouse2"),
                         (id==0?"lighthouse1":"lighthouse2"),RT_object2lighthouse))
            continue;

        Vector4d lighthouse_pos(0,0,0,1);
        roboy_communication_middleware::DarkRoom msg;

        for(auto const &sensor:sensor_position){
            Vector4d sensor_pos;
            sensor_pos = RT_object2lighthouse*sensor.second;
            double distance = sqrt(pow(sensor_pos[0],2.0)+pow(sensor_pos[1],2.0)+pow(sensor_pos[2],2.0));
            double elevation = 180.0-acos(sensor_pos[2]/distance)*180.0/M_PI;
            double azimuth = atan2(sensor_pos[1],sensor_pos[0])*180.0/M_PI;

            if(elevation>=0 && elevation<=180.0 && azimuth >=0 && azimuth <= 180.0){ // if the sensor is visible by lighthouse
                uint32_t sensor_value;
                if(!angle_switch){
                    uint32_t elevation_in_ticks = (uint32_t)(degreesToTicks(elevation));
                    sensor_value = (uint32_t)(id<<31|1<<30|true<<29|elevation_in_ticks&0x1fffffff);
                    if(sensor.first == 0)
                        ROS_DEBUG_THROTTLE(1, "elevation: %lf in ticks: %d", elevation, elevation_in_ticks);
                }else{
                    uint32_t azimuth_in_ticks = (uint32_t)(degreesToTicks(azimuth));
                    sensor_value = (uint32_t)(id<<31|0<<30|true<<29|azimuth_in_ticks&0x1fffffff);
                    if(sensor.first == 0)
                        ROS_DEBUG_THROTTLE(1, "azimuth: %lf in ticks: %d", azimuth, azimuth_in_ticks);
                }

                msg.sensor_value.push_back(sensor_value);
                Vector3d pos(sensor_pos[0],sensor_pos[1],sensor_pos[2]);
                publishSphere(pos,(id==0?"lighthouse1":"lighthouse2"),"simulated_sensor_positions",
                              sensor.first+id*sensor_position.size()+6543, COLOR(0,1,0,1), 0.01);
            }else{
                Vector3d pos(sensor_pos[0],sensor_pos[1],sensor_pos[2]);
                publishSphere(pos,(id==0?"lighthouse1":"lighthouse2"),"simulated_sensor_positions",
                              sensor.first+id*sensor_position.size()+7543, COLOR(1,0,0,1));
            }
//            Vector3d origin(0,0,0);
//            Vector3d dir(sensor_pos[0],sensor_pos[1],sensor_pos[2]);
//            publishRay(origin, dir,(id==0?"lighthouse1":"lighthouse2"),"simulated_sensor_rays",
//                       sensor.first+class_counter*sensor_position.size()+8543, COLOR(0,0,0,1), 0.01);
        }
        angle_switch = !angle_switch;

        if(!msg.sensor_value.empty())
            sensors_pub.publish(msg);

        rate.sleep();
    }
}