#include <gazebo_range_sensor_plugin.h>

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(RangesPlugin)

    RangesPlugin::RangesPlugin() : ModelPlugin() {}

    RangesPlugin::~RangesPlugin() { update_connection_->~Connection(); }

    void RangesPlugin::getSdfParams(sdf::ElementPtr sdf) {
        namespace_.clear();
        if (sdf->HasElement("robotNamespace")) {
            namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
        }
        if (sdf->HasElement("pubRate")) {
            pub_rate_ = sdf->GetElement("pubRate")->Get<double>();
        } else {
            pub_rate_ = kDefaultPubRate;
            gzwarn << "[ranges_plugin] Using default publication rate of "
                   << pub_rate_ << "Hz\n";
        }
        if (sdf->HasElement("rangesTopic")) {
            ranges_topic_ = sdf->GetElement("rangesTopic")->Get<std::string>(); 
        } else {
            ranges_topic_ = kDefaultRangesTopic; 
        }
        if (sdf->HasElement("noise")) {
            ranges_noise_ = sdf->GetElement("noise")->Get<double>();
        } else {
            ranges_noise_ = kDefaultRangesNoise;
            gzwarn << "[ranges_plugin] Using default noise "
                   << ranges_noise_ << "\n";
        }
        if (sdf->HasElement("fov")) {
            // we'll check visibility using half the viewing angle
            max_angle_ = sdf->GetElement("fov")->Get<double>()/2.0;
        } else {
            max_angle_ = kDefaultFov/2.0;
            gzwarn << "[ranges_plugin] Using default viewing angle "
                   << kDefaultFov << "\n"; } 
        if (sdf->HasElement("dropProb")) {
            drop_prob_ = sdf->GetElement("dropProb")->Get<double>();
        } else {
            drop_prob_ = kDefaultDropProb;
            gzwarn << "[ranges_plugin] Using default probability "
                   << kDefaultDropProb << " for dropping measurements\n";
        }
    }

    void RangesPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        getSdfParams(sdf);
        model_ = model;
        world_ = model_->GetWorld();
        last_time_ = world_->SimTime();
        last_pub_time_ = world_->SimTime();

        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("ROS node for gazebo not initialized");
            return;
        }
        node_handle_ = new ros::NodeHandle(namespace_);

        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&RangesPlugin::OnUpdate, this, _1));

        ranges_pub_ =
            node_handle_->advertise<range_sensor::RangeMeasurementArray>(ranges_topic_, 1);
        
        initialized_ = false;
    }

    void RangesPlugin::OnUpdate(const common::UpdateInfo &)
    {
        common::Time current_time = world_->SimTime();
        double dt = (current_time - last_pub_time_).Double();

        if (initialized_ == false) {
            if (world_->ModelByName("apriltag_tank")->GetChildLink("tag_1::base_link")) {
                pos_tag_1_ = world_->ModelByName("apriltag_tank")->GetChildLink("tag_1::base_link")->RelativePose().Pos();
                gzmsg << "[ranges plugin] Tag 1 Position found.\n";
            } else {
                gzwarn << "[ranges_plugin] Tag 1 Position not found.\n";
            }
            if (world_->ModelByName("apriltag_tank")->GetChildLink("tag_2::base_link")) {
                pos_tag_2_ = world_->ModelByName("apriltag_tank")->GetChildLink("tag_2::base_link")->RelativePose().Pos();
                gzmsg << "[ranges plugin] Tag 2 Position found.\n";
            } else {
                gzwarn << "[ranges_plugin] Tag 2 Position not found.\n";
            }
            if (world_->ModelByName("apriltag_tank")->GetChildLink("tag_3::base_link")) {
                pos_tag_3_ = world_->ModelByName("apriltag_tank")->GetChildLink("tag_3::base_link")->RelativePose().Pos();
                gzmsg << "[ranges plugin] Tag 3 Position found.\n";
            } else {
                gzwarn << "[ranges_plugin] Tag 3 Position not found.\n";
            }
            if (world_->ModelByName("apriltag_tank")->GetChildLink("tag_4::base_link")) {
                pos_tag_4_ = world_->ModelByName("apriltag_tank")->GetChildLink("tag_4::base_link")->RelativePose().Pos();
                gzmsg << "[ranges plugin] Tag 4 Position found.\n";
                initialized_ = true;
            } else {
                gzwarn << "[ranges_plugin] Tag 4 Position not found.\n";
            }
        }

        if (dt > 1.0 / pub_rate_ && initialized_ == true)
        {
            range_sensor::RangeMeasurementArray msg_array;
            msg_array.header.stamp = ros::Time::now();
            msg_array.header.frame_id = "map";

            // get world pose, orientation
            ignition::math::Vector3d pos_sensor =
                model_->GetLink("range_sensor_link")->WorldPose().Pos();

            ignition::math::Vector3d x_unit_vector(1.0, 0.0, 0.0);
            ignition::math::Vector3d body_x_axis = model_->GetLink("range_sensor_link")->WorldPose().Rot().RotateVector(x_unit_vector);
            
            // get viewing angle of tag
            ignition::math::Vector3d sensor_to_tag_1 = pos_tag_1_ - pos_sensor;
            double angle_tag_1 = acos(sensor_to_tag_1.Dot(body_x_axis)) / (sensor_to_tag_1.Length() * body_x_axis.Length());
            
            // probability that measurement will be dropped
            double p1 = standard_normal_distribution_(random_generator_);
            if (angle_tag_1 < max_angle_ && p1 >= drop_prob_) {
                range_sensor::RangeMeasurement msg;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "map";
                msg.id = 1;
                // still need to add noise
                msg.range = sensor_to_tag_1.Length();
                msg_array.measurements.push_back(msg);
            }
            
            ignition::math::Vector3d sensor_to_tag_2 = pos_tag_2_ - pos_sensor;
            double angle_tag_2 = acos(sensor_to_tag_2.Dot(body_x_axis)) / (sensor_to_tag_2.Length() * body_x_axis.Length());
            
            // probability that measurement will be dropped
            double p2 = standard_normal_distribution_(random_generator_);
            if (angle_tag_2 < max_angle_ && p2 >= drop_prob_) {
                range_sensor::RangeMeasurement msg;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "map";
                msg.id = 2;
                // still need to add noise
                msg.range = sensor_to_tag_2.Length();
                msg_array.measurements.push_back(msg);
            }

            ignition::math::Vector3d sensor_to_tag_3 = pos_tag_3_ - pos_sensor;
            double angle_tag_3 = acos(sensor_to_tag_3.Dot(body_x_axis)) / (sensor_to_tag_3.Length() * body_x_axis.Length());
            
            // probability that measurement will be dropped
            double p3 = standard_normal_distribution_(random_generator_);
            if (angle_tag_3 < max_angle_ && p3 >= drop_prob_) {
                range_sensor::RangeMeasurement msg;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "map";
                msg.id = 3;
                // still need to add noise
                msg.range = sensor_to_tag_3.Length();
                msg_array.measurements.push_back(msg);
            }

            ignition::math::Vector3d sensor_to_tag_4 = pos_tag_4_ - pos_sensor;
            double angle_tag_4 = acos(sensor_to_tag_4.Dot(body_x_axis)) / (sensor_to_tag_4.Length() * body_x_axis.Length());
            
            double p4 = standard_normal_distribution_(random_generator_);
            if (angle_tag_4 < max_angle_ && p4 >= drop_prob_) {
                range_sensor::RangeMeasurement msg;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "map";
                msg.id = 4;
                // still need to add noise
                msg.range = sensor_to_tag_4.Length();
                msg_array.measurements.push_back(msg);
            }
            
            ranges_pub_.publish(msg_array);
            last_pub_time_ = current_time;
        }
    }

} // namespace gazebo
