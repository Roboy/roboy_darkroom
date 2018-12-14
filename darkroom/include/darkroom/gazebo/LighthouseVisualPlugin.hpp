#pragma once

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <gazebo_msgs/ModelStates.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <common_utilities/rviz_visualization.hpp>


using namespace std;

namespace gazebo
{
    namespace rendering
    {
        class LighthouseVisualPlugin : public VisualPlugin, rviz_visualization
        {
        public:
            /// \brief Constructor
            LighthouseVisualPlugin();

            /// \brief Destructor
            virtual ~LighthouseVisualPlugin();

            void Load( VisualPtr _parent, sdf::ElementPtr _sdf );


        protected:
            /// \brief Update the visual plugin
            virtual void UpdateChild();


        private:
            /// \brief pointer to ros node
            ros::NodeHandlePtr nh;

            static int instance_counter;
            char name[30];

            // /// \brief The visual pointer used to visualize the force.
            VisualPtr visual_;

            // /// \brief The scene pointer.
            ScenePtr scene_;

            /// \brief For example a line to visualize the force
            DynamicLines *line;

            // Pointer to the update event connection
            event::ConnectionPtr update_connection_;

            tf::Transform pose;
        };
    }
}