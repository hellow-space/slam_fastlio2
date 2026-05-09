#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <pcl/search/kdtree.h>
#include <Eigen/Dense>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <limits>

/**
 * @brief 障碍物信息结构体
 * 存储每个障碍物的中心点、包围盒、距离、角度、点数量
 */
struct ObstacleInfo {
  Eigen::Vector4f centroid;    // 中心点坐标
  Eigen::Vector4f min_pt;      // 包围盒最小点
  Eigen::Vector4f max_pt;      // 包围盒最大点
  float distance;             // 障碍物到雷达的水平距离
  float angle_deg;             // 障碍物方位角（度）
  int point_count;             // 障碍物点云数量
};

/**
 * @class ObstacleAvoidanceNode
 * @brief 基于激光雷达点云的3D障碍物检测与避障节点（ROS2）
 * 功能：点云滤波 → 地面分割 → 障碍物聚类 → 状态判断 → 结果发布
 */
class ObstacleAvoidanceNode : public rclcpp::Node {
public:
  /**
   * @brief 构造函数：初始化参数、订阅者、发布者
   */
  ObstacleAvoidanceNode() : Node("obstacle_avoidance_node") {
    // 加载参数
    loadParams();

    // 订阅Livox雷达原始点云
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar", 10,
    std::bind(&ObstacleAvoidanceNode::cloudCallback, this, std::placeholders::_1));

    // 发布处理后的点云
    pub_roi_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle_avoidance/cloud_roi", 10);
    pub_obstacle_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle_avoidance/cloud_obstacles", 10);
    pub_ground_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle_avoidance/cloud_ground", 10);
    pub_final_processed_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/processed_final", 10);

    // 发布可视化、状态、障碍物位姿
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_avoidance/markers", 10);
    pub_status_ = this->create_publisher<std_msgs::msg::String>("/obstacle_avoidance/status", 10);
    pub_obstacle_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/obstacle_avoidance/obstacle_poses", 10);

    RCLCPP_INFO(this->get_logger(), "✅ 障碍物避障节点启动完成 (ROS2)");
  }

private:
  // ====================== 可配置参数（从yaml/命令行读取）======================
  double min_range_;            // 最小检测距离
  double max_range_;            // 最大检测距离
  double angle_deg_;            // 前方扇形角度
  double ground_plane_dist_thresh_; // 地面拟合阈值

  double roi_x_min_, roi_x_max_;    // X方向ROI（前后）
  double roi_y_min_, roi_y_max_;    // Y方向ROI（左右）
  double roi_z_min_, roi_z_max_;    // Z方向ROI（高低）
  double ground_z_max_;             // 地面最大高度
  double voxel_size_;               // 体素降采样大小
  double cluster_tolerance_;        // 聚类距离阈值
  double min_obstacle_height_;      // 最小障碍物高度
  int min_cluster_size_;            // 最小聚类点数
  int max_cluster_size_;            // 最大聚类点数
  double emergency_dist_;           // 紧急停止距离
  double danger_dist_;              // 危险减速距离
  double warning_dist_;             // 警告距离

  /**
   * @brief 加载ROS2参数（默认值 + 外部配置）
   */
  void loadParams() {
    // 声明参数（带默认值）
    this->declare_parameter<double>("roi_x_min", 0.0);
    this->declare_parameter<double>("roi_x_max", 15.0);
    this->declare_parameter<double>("roi_y_min", -5.0);
    this->declare_parameter<double>("roi_y_max", 5.0);
    this->declare_parameter<double>("roi_z_min", -0.3);
    this->declare_parameter<double>("roi_z_max", 2.5);
    this->declare_parameter<double>("ground_z_max", 0.15);
    this->declare_parameter<double>("voxel_size", 0.08);
    this->declare_parameter<double>("cluster_tolerance", 0.5);
    this->declare_parameter<int>("min_cluster_size", 10);
    this->declare_parameter<int>("max_cluster_size", 25000);
    this->declare_parameter<double>("emergency_distance", 1.5);
    this->declare_parameter<double>("danger_distance", 3.0);
    this->declare_parameter<double>("warning_distance", 6.0);
    this->declare_parameter<double>("min_obstacle_height", 0.15);
    this->declare_parameter<double>("min_range", 0.1);
    this->declare_parameter<double>("max_range", 1.5);
    this->declare_parameter<double>("angle_deg", 60.0);
    this->declare_parameter<double>("ground_plane_dist_thresh", 0.15);

    // 获取参数到成员变量
    this->get_parameter("roi_x_min", roi_x_min_);
    this->get_parameter("roi_x_max", roi_x_max_);
    this->get_parameter("roi_y_min", roi_y_min_);
    this->get_parameter("roi_y_max", roi_y_max_);
    this->get_parameter("roi_z_min", roi_z_min_);
    this->get_parameter("roi_z_max", roi_z_max_);
    this->get_parameter("ground_z_max", ground_z_max_);
    this->get_parameter("voxel_size", voxel_size_);
    this->get_parameter("cluster_tolerance", cluster_tolerance_);
    this->get_parameter("min_cluster_size", min_cluster_size_);
    this->get_parameter("max_cluster_size", max_cluster_size_);
    this->get_parameter("emergency_distance", emergency_dist_);
    this->get_parameter("danger_distance", danger_dist_);
    this->get_parameter("warning_distance", warning_dist_);
    this->get_parameter("min_obstacle_height", min_obstacle_height_);
    this->get_parameter("min_range", min_range_);
    this->get_parameter("max_range", max_range_);
    this->get_parameter("angle_deg", angle_deg_);
    this->get_parameter("ground_plane_dist_thresh", ground_plane_dist_thresh_);
  }

  /**
   * @brief 点云回调函数（核心处理流程）
   * @param msg 原始点云消息
   */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 1. 将ROS点云转为PCL点云格式
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *raw_cloud);

    // 空点云直接返回安全状态
    if (raw_cloud->empty()) {
      publishSafeStatus();
      return;
    }

    // 2. ROI区域过滤（只保留前方有效区域）
    pcl::PointCloud<pcl::PointXYZI>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    applyROIFilter(raw_cloud, roi_cloud);
    if (roi_cloud->empty()) {
      publishSafeStatus();
      return;
    }

    // 3. 体素降采样（减少点数量，提升运行速度）
    pcl::PointCloud<pcl::PointXYZI>::Ptr down_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    voxelDownsample(roi_cloud, down_cloud);
    publishCloud(pub_roi_cloud_, down_cloud, msg->header);

    // 4. 地面分割（分离地面点和障碍物点）
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    removeGround(down_cloud, ground_cloud, obstacle_cloud);

    // 发布地面、障碍物、最终点云
    publishCloud(pub_ground_cloud_, ground_cloud, msg->header);
    publishCloud(pub_obstacle_cloud_, obstacle_cloud, msg->header);
    publishCloud(pub_final_processed_, obstacle_cloud, msg->header);

    // 无障碍物 → 安全
    if (obstacle_cloud->empty()) {
      publishSafeStatus();
      return;
    }

    // 5. 欧式聚类（将点云分成一个个障碍物）
    std::vector<ObstacleInfo> obstacles;
    euclideanClustering(obstacle_cloud, obstacles);
    if (obstacles.empty()) {
      publishSafeStatus();
      return;
    }

    // 6. 计算最近障碍物距离
    float min_dist = std::numeric_limits<float>::max();
    for (const auto& obs : obstacles) {
      min_dist = std::min(min_dist, obs.distance);
    }

    // 7. 判断状态：紧急 / 危险 / 警告 / 安全
    std::string status;
    if (min_dist <= emergency_dist_)
      status = "EMERGENCY";
    else if (min_dist <= danger_dist_)
      status = "DANGER";
    else if (min_dist <= warning_dist_)
      status = "WARNING";
    else
      status = "SAFE";

    // 8. 发布所有结果
    publishStatus(status, min_dist, (int)obstacles.size());
    publishObstaclePoses(obstacles, msg->header);
    publishObstacleMarkers(obstacles, msg->header);
  }

  /**
   * @brief ROI扇形+高度区域过滤
   * 只保留：前方指定距离+角度+高度范围内的点
   */
  void applyROIFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr& output) {
    double angle_rad = angle_deg_ * M_PI / 180.0;
    output->clear();
    output->reserve(input->size());
    for (const auto& pt : input->points) {
      // 过滤无效点
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      // 极坐标距离与角度
      double r = std::hypot(pt.x, pt.y);
      double theta = std::atan2(pt.y, pt.x);
      // 满足范围才保留
      if (r >= min_range_ && r <= max_range_ &&
          std::fabs(theta) <= angle_rad &&
          pt.z >= roi_z_min_ && pt.z <= roi_z_max_) {
        output->push_back(pt);
      }
    }
  }

  /**
   * @brief 体素降采样：减少点云数量，加速计算
   */
  void voxelDownsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr& output) {
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(input);
    vg.setLeafSize((float)voxel_size_, (float)voxel_size_, (float)voxel_size_);
    vg.filter(*output);
  }

  /**
   * @brief 地面分割：使用RANSAC拟合平面，分离地面与障碍物
   */
  void removeGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr& ground,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacles) {
    ground->clear();
    obstacles->clear();
    if (input->empty()) return;

    // RANSAC平面拟合
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ground_plane_dist_thresh_);
    seg.setInputCloud(input);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    seg.segment(*inliers, *coeff);

    if (!inliers->indices.empty()) {
      // 提取地面
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      extract.setInputCloud(input);
      extract.setIndices(inliers);
      extract.setNegative(false); extract.filter(*ground);
      // 提取非地面
      extract.setNegative(true);
      pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground(new pcl::PointCloud<pcl::PointXYZI>);
      extract.filter(*non_ground);
      // 非地面点 → 障碍物
      for (const auto& pt : non_ground->points) {
        if (pt.z <= roi_z_max_) obstacles->push_back(pt);
      }
    } else {
      // 拟合失败 → 按高度简单区分地面/障碍物
      for (const auto& pt : input->points) {
        if (pt.z <= ground_z_max_) ground->push_back(pt);
        else if (pt.z <= roi_z_max_) obstacles->push_back(pt);
      }
    }
  }

  /**
   * @brief 欧式聚类：把连续点云分成独立障碍物
   */
  void euclideanClustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
                           std::vector<ObstacleInfo>& obstacles) {
    obstacles.clear();
    if (input->empty()) return;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(input);
    std::vector<pcl::PointIndices> cluster_indices;

    // 聚类设置
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(static_cast<float>(cluster_tolerance_));
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(cluster_indices);

    // 遍历每个聚类，计算障碍物信息
    for (const auto& indices : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::copyPointCloud(*input, indices, *cluster);

      // 计算中心点、包围盒
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cluster, centroid);
      Eigen::Vector4f min_pt, max_pt;
      pcl::getMinMax3D(*cluster, min_pt, max_pt);

      // 高度过滤：太矮不算障碍物
      float obstacle_height = max_pt[2] - min_pt[2];
      if (obstacle_height < min_obstacle_height_) continue;

      // 计算朝向
      Eigen::Matrix3f covariance;
      pcl::computeCovarianceMatrixNormalized(*cluster, centroid, covariance);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(covariance);
      Eigen::Vector3f major_vec = eig.eigenvectors().col(2);
      double yaw = std::atan2(major_vec.y(), major_vec.x());

      // 填入障碍物信息
      ObstacleInfo info;
      info.centroid = centroid;
      info.min_pt = min_pt;
      info.max_pt = max_pt;
      info.distance = static_cast<float>(std::hypot(centroid[0], centroid[1]));
      info.angle_deg = static_cast<float>(yaw * 180.0 / M_PI);
      info.point_count = static_cast<int>(cluster->size());
      obstacles.push_back(info);
    }
  }

  /**
   * @brief 通用点云发布函数
   */
  template <typename PublisherPtr, typename Cloud>
  void publishCloud(PublisherPtr& pub, Cloud& cloud, std_msgs::msg::Header header) {
    if (pub->get_subscription_count() == 0) return;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header = header;
    pub->publish(msg);
  }

  /**
   * @brief 发布避障状态（JSON格式）
   */
  void publishStatus(const std::string& status, float min_dist, int cnt) {
    auto msg = std_msgs::msg::String();
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2)
       << "{\"status\":\"" << status
       << "\",\"min_distance\":" << min_dist
       << ",\"count\":" << cnt << "}";
    msg.data = ss.str();
    pub_status_->publish(msg);
  }

  /**
   * @brief 发布安全状态
   */
  void publishSafeStatus() {
    publishStatus("SAFE", 999.0, 0);
  }

  /**
   * @brief 发布障碍物包围盒（RViz可视化）
   */
  void publishObstacleMarkers(const std::vector<ObstacleInfo>& obs, std_msgs::msg::Header header) {
    visualization_msgs::msg::MarkerArray ma;
    int id = 0;
    for (const auto& o : obs) {
      visualization_msgs::msg::Marker m;
      m.header = header;
      m.ns = "obstacle_boxes";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      // 包围盒位置
      m.pose.position.x = (o.min_pt[0] + o.max_pt[0]) * 0.5f;
      m.pose.position.y = (o.min_pt[1] + o.max_pt[1]) * 0.5f;
      m.pose.position.z = (o.min_pt[2] + o.max_pt[2]) * 0.5f;
      m.pose.orientation.w = 1.0f;
      // 包围盒大小
      m.scale.x = std::max(0.1f, o.max_pt[0] - o.min_pt[0]);
      m.scale.y = std::max(0.1f, o.max_pt[1] - o.min_pt[1]);
      m.scale.z = std::max(0.1f, o.max_pt[2] - o.min_pt[2]);

      // 按距离颜色显示：红(紧急)→橙(危险)→黄(警告)→绿(安全)
      if (o.distance <= emergency_dist_) {
        m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f;
      } else if (o.distance <= danger_dist_) {
        m.color.r = 1.0f; m.color.g = 0.5f; m.color.b = 0.0f;
      } else if (o.distance <= warning_dist_) {
        m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f;
      } else {
        m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f;
      }
      m.color.a = 0.5f;
      ma.markers.push_back(m);
    }
    pub_markers_->publish(ma);
  }

  /**
   * @brief 发布障碍物中心点位姿
   */
  void publishObstaclePoses(const std::vector<ObstacleInfo>& obstacles, std_msgs::msg::Header header) {
    geometry_msgs::msg::PoseArray msg;
    msg.header = header;
    for (const auto& obs : obstacles) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = obs.centroid[0];
      pose.position.y = obs.centroid[1];
      pose.position.z = obs.centroid[2];
      // 朝向
      double yaw = obs.angle_deg * M_PI / 180.0;
      pose.orientation.z = sin(yaw * 0.5);
      pose.orientation.w = cos(yaw * 0.5);
      msg.poses.push_back(pose);
    }
    pub_obstacle_poses_->publish(msg);
  }

  // 订阅者与发布者
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_roi_cloud_, pub_obstacle_cloud_, pub_ground_cloud_, pub_final_processed_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_obstacle_poses_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleAvoidanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}