#pragma once

// Diagnostic-only: no ROS dependency, algorithm buffers, or writer thread.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <string>

namespace fastlivo_diag
{
inline double steadySeconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
constexpr double missing = std::numeric_limits<double>::quiet_NaN();

struct Row
{
  const char *event = "unspecified";  // Static literal, valid until buffered output.
  uint64_t loop_id = 0, frame_id = 0;
  double start_steady_s = 0, duration_ms = 0;
  double ros_s = missing, raw_stamp_s = missing, corrected_stamp_s = missing;
  double scan_end_s = missing, target_end_s = missing, callback_gap_ms = missing;
  double state_before_s = missing;
  int status = 0;
  int64_t imu_q = -1, lidar_q = -1, lidar_stamp_q = -1, prop_imu_q = -1;
  double imu_front_s = missing, imu_back_s = missing;
  double lidar_front_s = missing, lidar_back_s = missing;
  int64_t input_points = -1, down_points = -1, effective_points = -1;
  int64_t map_voxels = -1, path_poses = -1, iterations = -1;
  uint64_t count = 1;
};

class Recorder
{
public:
  explicit Recorder(const std::string &filename)
  {
    try
    {
      const auto parent = std::filesystem::path(filename).parent_path();
      if (!parent.empty()) std::filesystem::create_directories(parent);
      file_ = std::fopen(filename.c_str(), "wx"); // Never overwrite an earlier run.
      if (file_)
      {
        std::setvbuf(file_, file_buffer_.data(), _IOFBF, file_buffer_.size());
        std::fputs("event,loop_id,frame_id,start_steady_s,duration_ms,ros_s,raw_stamp_s,corrected_stamp_s,scan_end_s,target_end_s,state_before_s,age_s,header_age_s,corrected_header_age_s,callback_gap_ms,status,imu_q,lidar_q,lidar_stamp_q,prop_imu_q,imu_front_s,imu_back_s,lidar_front_s,lidar_back_s,input_points,down_points,effective_points,map_voxels,path_poses,iterations,count\n", file_);
      }
    }
    catch (const std::exception &) {} // Diagnostics must not prevent localization.
    if (!file_) std::fprintf(stderr, "[LIO diagnostics] cannot open %s; diagnostics disabled\n", filename.c_str());
  }
  ~Recorder() { close(); }
  Recorder(const Recorder &) = delete;
  Recorder &operator=(const Recorder &) = delete;
  bool enabled() const { return file_ != nullptr; }

  void beginInputLoop()
  {
    ++loop_id;
    // Inputs received now may belong to several queued scans, not the previous
    // processed frame nor necessarily the next frame selected for processing.
    frame_id = 0;
    scan_end_s = target_end_s = missing;
  }

  Row row(const char *event, double start = steadySeconds()) const
  {
    Row result;
    result.event = event;
    result.loop_id = loop_id;
    result.frame_id = frame_id;
    result.start_steady_s = start;
    result.scan_end_s = scan_end_s;
    result.target_end_s = target_end_s;
    return result;
  }
  void record(const Row &value)
  {
    if (!file_) return;
    rows_[size_++] = value;
    if (size_ == rows_.size()) flush();
  }
  void duration(const char *event, double start, double end)
  {
    auto value = row(event, start);
    value.duration_ms = (end - start) * 1000;
    record(value);
  }
  void close()
  {
    if (!file_) return;
    flush(false);
    if (!file_) return; // flush() may have disabled output after a write failure.
    if (std::fclose(file_) != 0) std::fprintf(stderr, "[LIO diagnostics] close/write failed\n");
    file_ = nullptr;
  }

  class Scope
  {
  public:
    Scope(Recorder &owner, const char *event, double start) : row(owner.row(event, start)), owner_(owner) {}
    ~Scope() { finish(); }
    Scope(const Scope &) = delete;
    Scope &operator=(const Scope &) = delete;
    void finish()
    {
      if (finished_) return;
      row.duration_ms = (steadySeconds() - row.start_steady_s) * 1000;
      owner_.record(row);
      finished_ = true;
    }
    Row row;
  private:
    Recorder &owner_;
    bool finished_ = false;
  };
  Scope scope(const char *event, double start = steadySeconds()) { return Scope(*this, event, start); }

  uint64_t loop_id = 0, frame_id = 0;
  double scan_end_s = missing, target_end_s = missing;
  bool odom_published = false;

private:
  void flush(bool measure = true)
  {
    if (!file_ || size_ == 0) return;
    const double start = steadySeconds();
    for (size_t i = 0; i < size_; ++i)
    {
      const auto &v = rows_[i];
      const double age = std::isfinite(v.scan_end_s) && v.scan_end_s >= 0 ? v.ros_s - v.scan_end_s : missing;
      const int result = std::fprintf(file_,
        "%s,%llu,%llu,%.9f,%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.6f,%d,%lld,%lld,%lld,%lld,%.9f,%.9f,%.9f,%.9f,%lld,%lld,%lld,%lld,%lld,%lld,%llu\n",
        v.event, (unsigned long long)v.loop_id, (unsigned long long)v.frame_id,
        v.start_steady_s, v.duration_ms, v.ros_s, v.raw_stamp_s, v.corrected_stamp_s,
        v.scan_end_s, v.target_end_s, v.state_before_s, age, v.ros_s - v.raw_stamp_s, v.ros_s - v.corrected_stamp_s, v.callback_gap_ms, v.status,
        (long long)v.imu_q, (long long)v.lidar_q, (long long)v.lidar_stamp_q, (long long)v.prop_imu_q,
        v.imu_front_s, v.imu_back_s, v.lidar_front_s, v.lidar_back_s,
        (long long)v.input_points, (long long)v.down_points, (long long)v.effective_points,
        (long long)v.map_voxels, (long long)v.path_poses, (long long)v.iterations, (unsigned long long)v.count);
      if (result < 0)
      {
        std::fprintf(stderr, "[LIO diagnostics] output failed; diagnostics disabled\n");
        std::fclose(file_);
        file_ = nullptr;
        size_ = 0;
        return;
      }
    }
    size_ = 0;
    if (measure)
    {
      auto value = row("diagnostic_flush", start);
      value.duration_ms = (steadySeconds() - start) * 1000;
      rows_[size_++] = value;
    }
  }
  std::FILE *file_ = nullptr;
  std::array<Row, 512> rows_{};
  std::array<char, 65536> file_buffer_{};
  size_t size_ = 0;
};
} // namespace fastlivo_diag
