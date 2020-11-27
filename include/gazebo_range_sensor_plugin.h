#pragma once
#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <sdf/sdf.hh>

namespace gazebo {
static constexpr auto kDefaultPubRate = 7.0;
static constexpr auto kDefaultRangesTopic = "tag_detections_sim";
static constexpr auto kDefaultRangesNoise = 100.0;    
}

class RangesPlugin : public SensorPlugin {
    public: 
     RangesPlugin();
     virtual ~RangesPlugin();

    protected:
     virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);
     virtual void OnUpdate(const common::UpdateInfo &);
     void gedSdfParams(sdf::ElementPtr sdf);

    private:
     std::string namespace_;
     physics::ModelPtr model_;
     physics::WorldPtr world_;
     event::ConnectionPtr update_connection_;
     std::string ranges_topic_;
     double ranges_noise_;

     ros::NodeHandle *node_handle_;
     ros::Publisher ranges_pub_;

     double pub_rate_;

     std::default_random_engine random_generator_;
     std::normal_distribution<doubl> standard_normal_distribution_;

     common::Time last_pub_time_;
     common::Time last_time_;

     ignition::maths::Vector3d pos_tag_1_;
     ignition::maths::Vector3d pos_tag_2_;
     ignition::maths::Vector3d pos_tag_3_;
     ignition::maths::Vector3d pos_tag_4_;
     

    
} // namespace gazebo