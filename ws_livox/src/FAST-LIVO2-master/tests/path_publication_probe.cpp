// Integration probe: invokes the real LIVMapper::publish_path(), without LIO inputs.
#include "LIVMapper.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

int main(int argc, char **argv)
{
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  const int steps = args.size() > 1 ? std::stoi(args[1]) : 75000;
  const int seed = args.size() > 2 ? std::stoi(args[2]) : 0;
  const int tick_ms = args.size() > 3 ? std::stoi(args[3]) : 0;
  const bool validate = args.size() > 4 ? std::stoi(args[4]) != 0 : true;
  rclcpp::init(argc, argv);
  int result = 0;
  {
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr unused;
    LIVMapper mapper(unused, "path_publication_probe", options);
    mapper.geoQuat.w = 1;
    mapper.pubPath = mapper.node->create_publisher<nav_msgs::msg::Path>("/path_probe", 10);
    size_t received = 0, max_received_size = 0;
    double last_header = 0, minimum_header_gap = 1e9;
    bool messages_valid = true;
    auto sub = mapper.node->create_subscription<nav_msgs::msg::Path>("/path_probe", 10,
      [&](nav_msgs::msg::Path::ConstSharedPtr msg) {
        ++received;
        max_received_size = std::max(max_received_size, msg->poses.size());
        const double header = stamp2Sec(msg->header.stamp);
        if (last_header != 0) minimum_header_gap = std::min(minimum_header_gap, header - last_header);
        last_header = header;
        if (validate)
        {
          messages_valid &= msg->poses.size() <= 2000 && !msg->poses.empty();
          messages_valid &= msg->header.frame_id == "camera_init";
          messages_valid &= msg->header.stamp == msg->poses.back().header.stamp;
          for (size_t i = 1; i < msg->poses.size(); ++i)
            messages_valid &= msg->poses[i].pose.position.x == msg->poses[i-1].pose.position.x + 1;
        }
      });
    mapper.path.poses.resize(seed);
    for (int i = 0; i < seed; ++i) mapper.path.poses[i].pose.position.x = i - seed;
    std::vector<double> durations;
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < steps; ++i)
    {
      const auto tick_start = std::chrono::steady_clock::now();
      mapper._state.pos_end.x() = i;
      mapper.publish_path(mapper.pubPath);
      durations.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tick_start).count());
      rclcpp::spin_some(mapper.node);
      if (validate && mapper.path.poses.size() > 2000)
      {
        std::cerr << "REGRESSION: history exceeds 2000 poses at input " << i << std::endl;
        result = 2;
        break;
      }
      if (tick_ms > 0) std::this_thread::sleep_until(tick_start + std::chrono::milliseconds(tick_ms));
    }
    if (result == 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1050));
      mapper._state.pos_end.x() = steps;
      mapper.publish_path(mapper.pubPath);
      const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
      while (std::chrono::steady_clock::now() < until)
      {
        rclcpp::spin_some(mapper.node);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      if (validate && !(mapper.path.poses.size() == 2000 &&
                        mapper.path.poses.front().pose.position.x == steps - 1999 &&
                        mapper.path.poses.back().pose.position.x == steps &&
                        received >= 2 && minimum_header_gap >= .95 && messages_valid))
      {
        std::cerr << "REGRESSION: retained order, cap, header, or publication cadence failed" << std::endl;
        result = 3;
      }
    }
    double sum = 0;
    for (double x : durations) sum += x;
    std::sort(durations.begin(), durations.end());
    const auto percentile = [&](double p) { return durations[std::ceil(p * durations.size()) - 1]; };
    std::cout << "PROBE_JSON {\"steps\":" << durations.size()
              << ",\"seed\":" << seed << ",\"mean_ms\":" << sum / durations.size()
              << ",\"p95_ms\":" << percentile(.95) << ",\"max_ms\":" << durations.back()
              << ",\"retained\":" << mapper.path.poses.size()
              << ",\"received\":" << received << ",\"max_received_size\":" << max_received_size
              << ",\"minimum_header_gap_s\":" << minimum_header_gap
              << ",\"elapsed_s\":" << std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()
              << ",\"result\":" << result << "}" << std::endl;
  }
  rclcpp::shutdown();
  return result;
}
