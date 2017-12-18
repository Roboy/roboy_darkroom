#include "darkroom/gazebo/LighthouseVisualPlugin.hpp"

namespace gazebo
{
    namespace rendering
    {


        int LighthouseVisualPlugin::instance_counter = 1;

        ////////////////////////////////////////////////////////////////////////////////
        // Constructor
        LighthouseVisualPlugin::LighthouseVisualPlugin():line(NULL)
        {

        }

        ////////////////////////////////////////////////////////////////////////////////
        // Destructor
        LighthouseVisualPlugin::~LighthouseVisualPlugin()
        {
        }

        ////////////////////////////////////////////////////////////////////////////////
        // Load the plugin
        void LighthouseVisualPlugin::Load( VisualPtr _parent, sdf::ElementPtr _sdf )
        {
            // start ros node
            if (!ros::isInitialized())
            {
                int argc = 0;
                char** argv = NULL;
                ros::init(argc,argv,"LighthouseVisualPlugin",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
            }

            nh = ros::NodeHandlePtr(new ros::NodeHandle);

            this->visual_ = _parent;

            visual_->MakeStatic();

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->update_connection_ = gazebo::event::Events::ConnectRender(boost::bind(&LighthouseVisualPlugin::UpdateChild, this));

            trans = boost::shared_ptr<DarkRoom::Transform>(new DarkRoom::Transform);

            sprintf(name, "lighthouse%d", instance_counter);
            instance_counter++;

//            line = new DynamicLines();
//            line->Init(RENDERING_LINE_STRIP);
//            line->GetPointCount();
////            // Add two points to a connecting line strip from link_pose to endpoint
//            line->AddPoint(ignition::math::Vector3<double>(0, 0, 0), common::Color::Purple);
//            line->AddPoint(ignition::math::Vector3<double>(1, 1, 1), common::Color::Purple);
//////                // set the Material of the line, in this case to purple
////            line->setMaterial("Gazebo/Purple");
//            line->setVisibilityFlags(GZ_VISIBILITY_GUI);

            ROS_INFO("Ligthouse plugin loaded");
        }

        //////////////////////////////////////////////////////////////////////////////////
        // Update the visualizer
        void LighthouseVisualPlugin::UpdateChild()
        {
            ros::spinOnce();

            if(trans->getTransform(name,"world",pose)){
                tf::Vector3 origin = pose.getOrigin();
                tf::Quaternion orientation = pose.getRotation();
                math::Pose p(origin.getX(), origin.getY(), origin.getZ(), 0, 0, 0);
                visual_->SetWorldPose(p);


            }else{
                ROS_INFO_STREAM_THROTTLE(1, "no tf for " << name );
            }
//
        }

//        //////////////////////////////////////////////////////////////////////////////////
//        // VisualizeForceOnLink
//        void LighthouseVisualPlugin::VisualizeForceOnLink(const geometry_msgs::PointConstPtr &force_msg)
//        {
//            this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
//
//

//        }

        // Register this plugin within the simulator
        GZ_REGISTER_VISUAL_PLUGIN(LighthouseVisualPlugin)
    }
}