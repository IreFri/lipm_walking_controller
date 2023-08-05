#pragma once

#include <mc_observers/Observer.h>
#include <thread>
#include <mutex>
#include <memory>

namespace lipm_walking
{

using duration_ms = std::chrono::duration<double, std::milli>;
using duration_us = std::chrono::duration<double, std::micro>;

/** mc_rtc::clock is a clock that is always steady and thus suitable for performance measurements */
using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                        std::chrono::high_resolution_clock,
                                        std::chrono::steady_clock>::type;

struct CameraSensor : public mc_observers::Observer
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  CameraSensor(const std::string & type, double dt);

  virtual ~CameraSensor()
  {
    stop_loop_ = true;
    loop_sensor_.join();
  }

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void update(mc_control::MCController & ctl) override;

public:

protected:
  /*! \brief Add observer from logger
   *
   * @param category Category in which to log this observer
   */
  void addToLogger(const mc_control::MCController &, mc_rtc::Logger &, const std::string & category) override;

  /*! \brief Remove observer from logger
   *
   * @param category Category in which this observer entries are logged
   */
  void removeFromLogger(mc_rtc::Logger &, const std::string & category) override;

  /*! \brief Add observer information the GUI.
   *
   * @param category Category in which to add this observer
   */
  void addToGUI(const mc_control::MCController &,
                mc_rtc::gui::StateBuilder &,
                const std::vector<std::string> & /* category */) override;

protected:
  /// @{
  std::string robot_name_ = ""; ///< Name of robot to which the rnage sensor belongs

  std::string sensor_name_ = "";

  std::string path_to_preset_ = "";

  std::string camera_serial_ = "";

  std::string path_to_replay_data_ = "";

  size_t kernel_size_ = 3;

  float kernel_threshold_ = 0.005f;

  float outlier_threshold_ = 0.01f;

  //
  void startReadingCamera();

  std::thread loop_sensor_; ///< Thread to read the sensor data asynchronously
  std::atomic<bool> stop_loop_{false};
  std::atomic<bool> new_data_{false};
  std::mutex mutex_;
  std::vector<Eigen::Vector3d> points_;
  double t_ = 0.; // controller time
};

} /* lipm_walking */