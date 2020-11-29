#pragma once
#include <ros/ros.h>
#include <range_sensor/RangeMeasurement.h>
#include <range_sensor/RangeMeasurementArray.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <sdf/sdf.hh>


namespace gazebo {
static constexpr auto kDefaultPubRate = 7.0;
static constexpr auto kDefaultRangesTopic = "tag_detections_sim";
static constexpr auto kDefaultRangesNoise = 0.1;
static constexpr auto kDefaultFov = 70;
static constexpr auto kDefaultDropProb = 0.05;



class RangesPlugin : public ModelPlugin {
    public: 
     RangesPlugin();
     virtual ~RangesPlugin();

    protected:
     virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
     virtual void OnUpdate(const common::UpdateInfo &);
     void getSdfParams(sdf::ElementPtr sdf);

    private:
     std::string namespace_;
     physics::ModelPtr model_;
     physics::WorldPtr world_;
     event::ConnectionPtr update_connection_;
     std::string ranges_topic_;
     double ranges_noise_;
     double max_angle_;
     double drop_prob_;

     ros::NodeHandle *node_handle_;
     ros::Publisher ranges_pub_;

     double pub_rate_;

     std::default_random_engine random_generator_;
     std::normal_distribution<double> standard_normal_distribution_;
     std::uniform_real_distribution<double> uniform_real_distribution_;

     common::Time last_pub_time_;
     common::Time last_time_;

     ignition::math::Vector3d pos_tag_1_;
     ignition::math::Vector3d pos_tag_2_;
     ignition::math::Vector3d pos_tag_3_;
     ignition::math::Vector3d pos_tag_4_;

     bool initialized_;
     
};
} // namespace gazebo
