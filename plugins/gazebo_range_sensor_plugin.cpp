#include <gazebo_range_sensor_plugin.h>

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(RangesPlugin)

    RangesPlugin::RangesPlugin() : SensorPlugin()

                                       RangesPlugin::~RangesPlugin()
    {
        update_connection_->~Connection();
    }

    void RangesPlugin::getSdfParams(sed::ElementPtr sdf)
    {
        namespace_.clear();
        if (sdf->HasElement("robotNamespace"))
        {
            namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
        }
        if (sdf->HasElement("pubRate"))
        {
            pub_rate_ = sdf->GetElement("pubRate")->Get<double>();
        }
        else
        {
            pub_rate_ = kDefaultPubRate;
            gzwarn << "[ranges_plugin] Using default publication rate of "
                   << pub_rate_ << "Hz\n";
        }
        if (sdf->HasElement("rangesTopic"))
        {
            ranges_topic_ = sdf->GetElement("rangesTopic")->Get<std::string>();
        }
        else
        {
            ranges_topic_ = kDefaultRangesTopic;
        }
        if (sdf->HasElement("noise"))
        {
            ranges_noise_ = sdf->GetElement("noise")->Get<double>();
        }
        else
        {
            ranges_noise_ = kDefaultRangesNoise;
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
            ROS_FATAL_STREAM("Ros node for gazebo not initialized");
            return;
        }
        node_handle_ = new ros::NodeHandle(namespace_);

        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&RangesPlugin::OnUpdate, this, _1));

        ranges_pub_ =
            node_handle_->advertise<range_sensor::RangeMeasurementArray>(ranges_topic_, 1);

        pos_tag_1_ = world_->ModelByName("tag_1")->RelativePose().Pos()
        pos_tag_2_ = world_->ModelByName("tag_2")->RelativePose().Pos()
        pos_tag_3_ = world_->ModelByName("tag_3")->RelativePose().Pos()
        pos_tag_4_ = world_->ModelByName("tag_4")->RelativePose().Pos()
    }

    void RangesPlugin::OnUpdate(const common::UpdateInfo &)
    {
        common::Time current_time = world_->SimTime();
        double dt = (current_time - last_pub_time_).Double();

        if (dt > 1.0 / pub_rate_)
        {
            range_sensor::RangeMeasurementArray msg;

            // get world pose, orientation
            ignition::math::Vector3d pos_sensor =
                model_->GetLink("range_sensor_link")->WorldPose().Pos();

            ignition::math::Vector3d x_axis = model_->GetLink("range_sensor_link")->WorldPose().Rot().RotateVector(Vector3d(1.0, 0.0, 0.0));
            
            // if tag within cone, compute distance to this tag
            ignition::math::Vector3d sensor_to_tag_1 = pose_tag_1_ - pos_sensor;
            double angle_tag_1 = acos(sensor_to_tag_1.Dot(x_axis)) / (sensor_to_tag_1.Length() * x_axis.Length());
            

            
            ignition::math::vector3d sensor_to_tag_2 = pose_tag_2_ - pos_sensor;
            ignition::math::vector3d sensor_to_tag_3 = pose_tag_3_ - pos_sensor;
            ignition::math::vector3d sensor_to_tag_4 = pose_tag_4_ - pos_sensor;

            

            // there should be a certain probability that this distance measurement
            // is dropped, this could be correlated to distance to tag

            // if not dropped, fill out RangeMeasurement msg

            // repeat this for other three tags, can be hardcoded

            // fill RangeMeasurementArray msg

            // publish this

            ranges_pub_.publish(msg);
            last_pub_time_ = current_time;
        }
    }

} // namespace gazebo