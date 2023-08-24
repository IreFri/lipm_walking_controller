#pragma once

#include <mc_observers/Observer.h>

#include <lipm_walking/observer/CameraSensorShared.h>

#include <thread>

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
    if(estimation_loop_.joinable())
    {
      estimation_loop_.join();
    }
  }

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void update(mc_control::MCController & ctl) override;

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

  /// @{
  std::string robot_name_; ///< Name of robot to which the rnage sensor belongs

  std::string sensor_name_;

  std::string desired_state_;
  //
  CameraSensorShared * data_;

  bool serverOnline_ = false;
  double lastServerOfflineMessage_t_ = 0.0;

  std::atomic<bool> stop_loop_{false};
  double t_ = 0.; // controller time

  void startGroundEstimation(mc_control::MCController & ctl);

  std::thread estimation_loop_;

  std::atomic<bool> new_ground_data_{false};
  std::mutex points_mtx_;
  std::vector<Eigen::Vector3d> points_;

  void updateServerOnline();
};

} // namespace lipm_walking
