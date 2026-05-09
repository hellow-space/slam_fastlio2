#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "yaml-cpp/yaml.h"
#include <cmath>

using namespace std::chrono_literals;
#define DEG2RAD (M_PI / 180.0)

class LivoxBodyTfPublisher : public rclcpp::Node
{
public:
    LivoxBodyTfPublisher() : Node("livox_body_tf_node")
    {
        this->declare_parameter("yaml_path", "");
        std::string yaml_path = this->get_parameter("yaml_path").as_string();

        loadYaml(yaml_path);

        publisher_ = this->create_publisher<geometry_msgs::msg::Transform>("livox_body_position", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&LivoxBodyTfPublisher::publishTransform, this));

        RCLCPP_INFO(this->get_logger(), "雷达-IMU外参发布成功！");
        RCLCPP_INFO(this->get_logger(), "x:%.2f y:%.2f z:%.2f", tx_, ty_, tz_);
        RCLCPP_INFO(this->get_logger(), "R:%.1f P:%.1f Y:%.1f", roll_, pitch_, yaw_);
    }

private:
    void loadYaml(const std::string& path)
    {
        YAML::Node config = YAML::LoadFile(path);

        tx_ = config["livox_body_position"]["translation"]["x"].as<double>();
        ty_ = config["livox_body_position"]["translation"]["y"].as<double>();
        tz_ = config["livox_body_position"]["translation"]["z"].as<double>();

        roll_ = config["livox_body_position"]["rotation_rpy"]["roll"].as<double>();
        pitch_ = config["livox_body_position"]["rotation_rpy"]["pitch"].as<double>();
        yaw_ = config["livox_body_position"]["rotation_rpy"]["yaw"].as<double>();

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

    void publishTransform()
    {
        geometry_msgs::msg::Transform msg;
        msg.translation.x = tx_;
        msg.translation.y = ty_;
        msg.translation.z = tz_;
        msg.rotation.x = qx_;
        msg.rotation.y = qy_;
        msg.rotation.z = qz_;
        msg.rotation.w = qw_;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double tx_, ty_, tz_;
    double roll_, pitch_, yaw_;
    double qx_, qy_, qz_, qw_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxBodyTfPublisher>());
    rclcpp::shutdown();
    return 0;
}

