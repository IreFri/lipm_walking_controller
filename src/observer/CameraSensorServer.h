#pragma once

#include <lipm_walking/observer/CameraSensorShared.h>

#include <mc_rtc/Configuration.h>

#include <open3d/Open3D.h>

#include <thread>

namespace lipm_walking
{

struct CameraSensorServer
{
  /** Initialize the server and kernel configuration */
  CameraSensorServer(const char * name, const mc_rtc::Configuration & config);

  ~CameraSensorServer();

  /** Starts the acquisition thread and the computation thread
   *
   * Then run forever until stop is called
   */
  void run();

  void stop();

private:
  std::string name_;
  /** SHM */
  CameraSensorShared * data_;
  /** Acquisition and computation parameters */
  std::string camera_serial_;
  std::string path_to_preset_;
  std::string path_to_replay_data_;
  size_t kernel_size_ = 3;
  float kernel_threshold_ = 0.005f;
  float outlier_threshold_ = 0.01f;

  /** Acquisition and computation data */
  std::mutex points_mtx_;
  std::vector<Eigen::Vector3d> points_;
  std::shared_ptr<open3d::geometry::PointCloud> pc_estimated_ground_points_;
  std::shared_ptr<open3d::geometry::PointCloud> pc_transformed_estimated_ground_points_;
  std::shared_ptr<open3d::geometry::PointCloud> pc_full_ground_reconstructed_points_;
  std::vector<Eigen::Vector3d> corrected_ground_points_;
  std::vector<Eigen::Vector3d> ground_points_;

  std::atomic<bool> run_ = false;

  std::atomic<bool> pipeline_started_ = false;

  std::thread acquisition_th_;
  void acquisition();

  std::thread computation_th_;
  void computation();

  void do_computation();
};

} // namespace lipm_walking
