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

    interactive_marker_sub = nh->subscribe("/interactive_markers/feedback", 1,
                                           &LighthouseSimulator::interactiveMarkersFeedback, this);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    if (const char *env_p = getenv("DARKROOM_CALIBRATED_OBJECTS")) {
        string path = env_p;
        ROS_INFO_STREAM("using DARKROOM_CALIBRATED_OBJECTS: " << path);
        readConfig(path  + "/" +  "protoType3.yaml", objectID, name, mesh, calibrated_sensors, sensors);
    } else {
        ROS_ERROR("could not get DARKROOM_CALIBRATED_OBJECTS environmental variable");
        return;
    }

    for(auto &sensor:sensors){
        Vector3d rel_location;
        sensor.second.getRelativeLocation(rel_location);
        sensor_position[sensor.first] << rel_location[0], rel_location[1], rel_location[2], 1;
    }

    char str[100];
    sprintf(str, "%s_simulated_%d", name.c_str(), id);

    name = string(str);

    class_counter++;

    tf::Vector3 origin(0,1,0);
    make6DofMarker(false,visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,origin,
                   true,0.1,(id==0?"lighthouse1":"lighthouse2"), name.c_str(),"");
    relative_object_pose.setOrigin(origin);
    relative_object_pose.setRotation(tf::Quaternion(0,0,1,0));

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
        if(!getTransform(name.c_str(), (id==0?"lighthouse1":"lighthouse2"),RT_object2lighthouse))
            continue;

        Vector4d lighthouse_pos(0,0,0,1);
        roboy_communication_middleware::DarkRoom msg;

        for(auto const &sensor:sensor_position){
            Vector4d sensor_pos;
            sensor_pos = RT_object2lighthouse*sensor.second;
            double distance = sqrt(pow(sensor_pos[0],2.0)+pow(sensor_pos[1],2.0)+pow(sensor_pos[2],2.0));
            double elevation = 180.0-acos(sensor_pos[2]/distance)*180.0/M_PI;
            double azimuth = atan2(sensor_pos[1],sensor_pos[0])*180.0/M_PI;

            uint32_t sensor_value;
            if(elevation>=0 && elevation<=180.0 && azimuth >=0 && azimuth <= 180.0){ // if the sensor is visible by lighthouse

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
                Vector3d pos(sensor_pos[0],sensor_pos[1],sensor_pos[2]);
                publishSphere(pos,(id==0?"lighthouse1":"lighthouse2"),"simulated_sensor_positions",
                              sensor.first+id*sensor_position.size()+6543, COLOR(0,1,0,1), 0.01);
            }else{
                Vector3d pos(sensor_pos[0],sensor_pos[1],sensor_pos[2]);
                publishSphere(pos,(id==0?"lighthouse1":"lighthouse2"),"simulated_sensor_positions",
                              sensor.first+id*sensor_position.size()+7543, COLOR(1,0,0,1));
            }
            msg.sensor_value.push_back(sensor_value);
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

void LighthouseSimulator::interactiveMarkersFeedback(const visualization_msgs::InteractiveMarkerFeedback &msg){
    tf::Vector3 position(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
    tf::Quaternion orientation(msg.pose.orientation.x, msg.pose.orientation.y,
                               msg.pose.orientation.z, msg.pose.orientation.w);
    if(strcmp(msg.marker_name.c_str(),name.c_str())==0){
        relative_object_pose.setOrigin(position);
        relative_object_pose.setRotation(orientation);
    }
}