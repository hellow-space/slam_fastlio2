#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer() : Node("pointcloud_transformer_node")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10,
            std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar_transformed", 10);

        RCLCPP_INFO(this->get_logger(), "点云变换节点启动成功！");
        RCLCPP_INFO(this->get_logger(), "订阅: /livox/lidar (livox_frame)");
        RCLCPP_INFO(this->get_logger(), "发布: /livox/lidar_transformed (base_link)");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try
        {
            if (!tf_buffer_->canTransform("base_link", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1)))
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "⏳ 等待 TF 变换...");
                return;
            }

            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(*msg, transformed_cloud, tf_buffer_->lookupTransform("base_link", msg->header.frame_id, msg->header.stamp));

            publisher_->publish(transformed_cloud);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "❌ TF 变换失败: %s", ex.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudTransformer>());
    rclcpp::shutdown();
    return 0;
}

