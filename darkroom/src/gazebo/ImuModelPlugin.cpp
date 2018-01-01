#include "darkroom/gazebo/ImuModelPlugin.hpp"

ImuModelPlugin::ImuModelPlugin() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "ImuModelPlugin",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner.reset(new ros::AsyncSpinner(1));
    spinner->start();

    // imu_pub = nh->advertise<geometry_msgs::>("/roboy/middleware/MotorStatus", 1);
}

ImuModelPlugin::~ImuModelPlugin() {
    // motor_status_publishing = false;
    // if(motor_status_publisher!=nullptr)
    //     motor_status_publisher->join();
}

void ImuModelPlugin::Load(gazebo::physics::ModelPtr parent_, sdf::ElementPtr sdf_) {
    ROS_INFO("Loading ImuModelPlugin plugin");
    // Save pointers to the model
    parent_model = parent_;
    sdf = sdf_;

    // Error message if the model couldn't be found
    if (!parent_model) {
        ROS_ERROR_STREAM_NAMED("loadThread", "parent model is NULL");
        return;
    }

    // Check that ROS has been initialized
    if (!ros::isInitialized()) {
        ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin.");
        return;
    }

    // Get all link and joint names
    physics::Link_V links = parent_model->GetLinks();
    for (auto link : links) {
        string link_name = link->GetName();
        link_names.push_back(link_name);
    }
    ROS_INFO("Parsing Imu Plugin");
    if (!parseImuSDF(sdf_->ToString(""), imus)){
        ROS_ERROR("ERROR parsing Imu Plugin, check your sdf file.");
        return;
    }

    for(ImuInfo imu:imus){
        imu_pub.push_back(nh->advertise<sensor_msgs::Imu>(("/roboy/middleware/"+imu.link->GetName()+"/imu").c_str(), 1));
    }


    physics::PhysicsEnginePtr physics_engine = parent_model->GetWorld()->GetPhysicsEngine();

    // Get the Gazebo solver type
    std::string solver_type = boost::any_cast<std::string>(physics_engine->GetParam("solver_type"));

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&ImuModelPlugin::Update, this));

    // motor_status_publisher.reset(new boost::thread(&ImuModelPlugin::MotorStatusPublisher,this));
    // motor_status_publisher->detach();

    t0 = high_resolution_clock::now();

    ROS_INFO("ImuModelPlugin ready");
}

void ImuModelPlugin::Update() {
    // Get the simulation time and period
    gz_time_now = parent_model->GetWorld()->GetSimTime();
    ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
    ros::Duration update_period = sim_time_ros - last_update_sim_time_ros;

    t1 = high_resolution_clock::now();
    microseconds time_span = duration_cast<microseconds>(t1-t0);

    if(time_span.count()>100000){ // 10Hz
        readSim(sim_time_ros, update_period);
        last_update_sim_time_ros = sim_time_ros;
        t0 = t1;
    }

//    writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros);

    last_write_sim_time_ros = sim_time_ros;
}

void ImuModelPlugin::readSim(ros::Time time, ros::Duration period) {
    ROS_DEBUG("read simulation");
    int i=0;
    for(ImuInfo imu:imus){
        switch(imu.type){
            case ACC:{
                math::Vector3 acc = imu.link->GetWorldLinearAccel();
                sensor_msgs::Imu msg;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = imu.link->GetName();
                msg.orientation.w = 1;
                msg.linear_acceleration_covariance = {
                        1, 0, 0,
                        0, 1, 0,
                        0, 0, 1
                };

                msg.linear_acceleration.x = acc.x;
                msg.linear_acceleration.y = acc.y;
                msg.linear_acceleration.z = acc.z;

                imu_pub[i].publish(msg);
                ROS_DEBUG_THROTTLE(1,"%f %f %f", acc[0], acc[1], acc[2]);
                break;
            }
            case GYRO:{
                ROS_WARN_ONCE("IMU type GYRO not implemented");
                break;
            }
            case IMU:{
                ROS_WARN_ONCE("IMU type IMU not implemented");
                break;
            }
        }
        i++;
    }
}

void ImuModelPlugin::writeSim(ros::Time time, ros::Duration period) {
    ROS_DEBUG("write simulation");
}

void ImuModelPlugin::Reset() {
    // Reset timing variables to not pass negative update periods to controllers on world reset
    last_update_sim_time_ros = ros::Time();
    last_write_sim_time_ros = ros::Time();
    //reset acceleration of links and the actuator simulation.
}

bool ImuModelPlugin::parseImuSDF(const string &sdf, vector<ImuInfo> &imu){
    // initialize TiXmlDocument doc with a string
    TiXmlDocument doc;
    if (!doc.Parse(sdf.c_str()) && doc.Error()) {
        ROS_ERROR("Can't parse IMU plugin. Invalid imu description.");
        return false;
    }

    // Find joints in transmission tags
    TiXmlElement *root = doc.RootElement();

    // Constructs the myoMuscles by parsing custom xml.
    TiXmlElement *imu_it = NULL;
    for (imu_it = root->FirstChildElement("link"); imu_it;
         imu_it = imu_it->NextSiblingElement("link")) {
        if (imu_it->Attribute("name")) {
            TiXmlElement *pose_child_it = NULL;
            for (pose_child_it = imu_it->FirstChildElement("pose"); pose_child_it;
                 pose_child_it = pose_child_it->NextSiblingElement("pose")) {
                 ImuInfo info;
                 info.link = parent_model->GetLink(imu_it->Attribute("name"));
                 float x, y, z, roll, pitch, yaw;
                 if (sscanf(pose_child_it->GetText(), "%f %f %f %f %f %f", &x, &y, &z, &roll, &pitch, &yaw) != 6) {
                     ROS_ERROR_STREAM_NAMED("parser", "error reading [pose] (x y z roll pitch yaw)");
                     return false;
                 }
                 info.position = Vector3d(x,y,z);
                 info.orientation = Vector3d(roll, pitch, yaw);
                 const char *type = pose_child_it->Attribute("type");
                 if (type){
                    if(strcmp(type, "ACC")==0){
                        info.type = ACC;
                    }else if(strcmp(type, "GYRO")==0){
                        info.type = GYRO;
                    }else if(strcmp(type, "IMU")==0){
                        info.type = IMU;
                    }
                 }
                 ROS_INFO("%s: %f %f %f %f %f %f", type, info.position[0], info.position[1], info.position[2],
                            info.orientation[0], info.orientation[1], info.orientation[2]);
                 imu.push_back(info);
             }
        }
    }
    return true;
}

GZ_REGISTER_MODEL_PLUGIN(ImuModelPlugin)
