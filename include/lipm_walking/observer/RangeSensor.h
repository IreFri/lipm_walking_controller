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

struct RangeSensor : public mc_observers::Observer
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  RangeSensor(const std::string & type, double dt);

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
  std::string range_sensor_name_ = ""; ///< Name of the sensor used
  std::string serial_port_name_ = ""; ///< Name of the serial port
  bool print_reading_error_once_ = true; ///< To keep track of the serial port error printing

  //
  void startReadingDevice();

  std::thread loop_sensor_; ///< Thread to read the sensor data asynchronously
  std::atomic<bool> debug_{false};
  std::atomic<bool> is_reading_{false};
  std::atomic<bool> serial_port_is_open_{false}; ///< To keep track of the serial port opening/closing
  std::atomic<double> sensor_data_ {0}; ///< Data read from the sensor
  std::atomic<double> measured_sensor_time_; ///< Measured the sensor elapsed time
  int baudrate_ = 0000015;
  clock::time_point prev_time_;
  duration_ms time_since_last_received_{0}; // time between two successful reading
  double timeout_ = 1000; // timeout in ms
};

} /* lipm_walking */