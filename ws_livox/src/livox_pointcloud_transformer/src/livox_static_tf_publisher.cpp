#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "yaml-cpp/yaml.h"
#include <cmath>

#define DEG2RAD (M_PI / 180.0)

class LivoxStaticTfPublisher : public rclcpp::Node
{
public:
    LivoxStaticTfPublisher() : Node("livox_static_tf_node")
    {
        this->declare_parameter("yaml_path", "");
        std::string yaml_path = this->get_parameter("yaml_path").as_string();

        loadYaml(yaml_path);

        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publishStaticTf();

        RCLCPP_INFO(this->get_logger(), "Livox->base_link 静态TF发布成功！");
        RCLCPP_INFO(this->get_logger(), "平移: x=%.2f y=%.2f z=%.2f", tx_, ty_, tz_);
        RCLCPP_INFO(this->get_logger(), "旋转: R=%.1f P=%.1f Y=%.1f", roll_, pitch_, yaw_);
    }

private:
    void loadYaml(const std::string& path)
    {
        YAML::Node config = YAML::LoadFile(path);

        tx_ = config["livox_to_base_link"]["translation"]["x"].as<double>();
        ty_ = config["livox_to_base_link"]["translation"]["y"].as<double>();
        tz_ = config["livox_to_base_link"]["translation"]["z"].as<double>();

        roll_ = config["livox_to_base_link"]["rotation_rpy"]["roll"].as<double>();
        pitch_ = config["livox_to_base_link"]["rotation_rpy"]["pitch"].as<double>();
        yaw_ = config["livox_to_base_link"]["rotation_rpy"]["yaw"].as<double>();

        double r = roll_ * DEG2RAD;
        double p = pitch_ * DEG2RAD;
        double y = yaw_ * DEG2RAD;

        double cy = cos(y * 0.5);
        double sy = sin(y * 0.5);
        double cp = cos(p * 0.5);
        double sp = sin(p * 0.5);
        double cr = cos(r * 0.5);
        double sr = sin(r * 0.5);

        qw_ = cy * cp * cr + sy * sp * sr;
        qx_ = cy * cp * sr - sy * sp * cr;
        qy_ = sy * cp * sr + cy * sp * cr;
        qz_ = sy * cp * cr - cy * sp * sr;
    }

    void publishStaticTf()
    {
        geometry_msgs::msg::TransformStamped tf_stamped;
        tf_stamped.header.stamp = this->now();
        tf_stamped.header.frame_id = "base_link";
        tf_stamped.child_frame_id = "livox_frame";

        tf_stamped.transform.translation.x = tx_;
        tf_stamped.transform.translation.y = ty_;
        tf_stamped.transform.translation.z = tz_;
        tf_stamped.transform.rotation.x = qx_;
        tf_stamped.transform.rotation.y = qy_;
        tf_stamped.transform.rotation.z = qz_;
        tf_stamped.transform.rotation.w = qw_;

        tf_broadcaster_->sendTransform(tf_stamped);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    double tx_, ty_, tz_;
    double roll_, pitch_, yaw_;
    double qx_, qy_, qz_, qw_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxStaticTfPublisher>());
    rclcpp::shutdown();
    return 0;
}

