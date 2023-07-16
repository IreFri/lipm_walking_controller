#pragma once

#include <lipm_walking/State.h>

#include <mc_rtc/ros.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <mutex>


// https://gist.github.com/edwintcloud/d547a4f9ccaf7245b06f0e8782acefaa
//===================================================================
// File: circular_buffer.cpp
//
// Desc: A Circular Buffer implementation in C++.
//
// Copyright © 2019 Edwin Cloud. All rights reserved.
//
//===================================================================

//-------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------
#include <memory>

//-------------------------------------------------------------------
// Circular_Buffer (Class)
//     We will implement the buffer with a templated class so
//     the buffer can be a buffer of specified type.
//-------------------------------------------------------------------
template <class T> class Circular_Buffer {
private:
  //---------------------------------------------------------------
  // Circular_Buffer - Private Member Variables
  //---------------------------------------------------------------

  std::shared_ptr<T[]> buffer; // using a smart pointer is safer (and we don't
                               // have to implement a destructor)
  size_t head = 0;             // size_t is an unsigned long
  size_t tail = 0;
  size_t max_size;
  T empty_item; // we will use this to clear data
public:
  //---------------------------------------------------------------
  // Circular_Buffer - Public Methods
  //---------------------------------------------------------------
  Circular_Buffer<T>() {}

  // Create a new Circular_Buffer.
  Circular_Buffer<T>(size_t max_size)
      : buffer(std::shared_ptr<T[]>(new T[max_size])), max_size(max_size){};

  // Add an item to this circular buffer.
  void enqueue(T item) {
    // if buffer is full, throw an error
    if (is_full())
    {
      dequeue();
    }

    // insert item at back of buffer
    buffer[tail] = item;

    // increment tail
    tail = (tail + 1) % max_size;
  }

  // Remove an item from this circular buffer and return it.
  T dequeue() {

    // if buffer is empty, throw an error
    if (is_empty() && max_size != 1)
    {
      return sva::PTransformd();
    }

    // get item at head
    T item = buffer[head];

    // set item at head to be empty
    T empty;
    buffer[head] = empty_item;

    // move head foward
    head = (head + 1) % max_size;

    // return item
    return item;
  }

  // Return the item at the front of this circular buffer.
  T front() { return buffer[head]; }

  // Return true if this circular buffer is empty, and false otherwise.
  bool is_empty() { return head == tail; }

  // Return true if this circular buffer is full, and false otherwise.
  bool is_full() { return tail == (head - 1) % max_size; }

  // Return the size of this circular buffer.
  size_t size() {
    if (tail >= head)
      return tail - head;
    return max_size - head - tail;
  }
};

namespace lipm_walking::states
{

enum class Foot
{
  //! Left foot
  Left = 0,

  //! Right foot
  Right
};




/** \brief FSM state to calculate ground profile and calculate sole stiffness online. */
struct SoftFootState : State
{
public:
  void configure(const mc_rtc::Configuration & config) override
  {
    config_.load(config);
  }

  /** \brief Start. */
  void start() override;

  /** \brief checkTransitions. */
  bool checkTransitions() override;

  /** \brief runState. */
  void runState() override;

  /** \brief Teardown. */
  void teardown() override;

protected:

  void calculateCost(mc_control::fsm::Controller & ctl);

  void estimateGround(mc_control::fsm::Controller & ctl, const Foot & current_moving_foot);

  void extractGroundSegment(mc_control::fsm::Controller & ctl, const Foot & current_moving_foot, const Eigen::Vector3d & landing);

  void extractAltitudeProfileFromGroundSegment(const Foot & current_moving_foot);

  void updateVariableStiffness(mc_control::fsm::Controller & ctl, const Foot & current_moving_foot);

  void computeSegmentConvexHull(mc_control::fsm::Controller & ctl, const Foot & current_moving_foot);

  std::vector<Eigen::Vector3d> computeConvexHull(const std::vector<Eigen::Vector3d>& data);

  void computeMinMaxAngle(const Foot & current_moving_foot);

  void computeFootLandingPosition(const Foot & current_moving_foot, const Eigen::Vector3d & landing);

  void computeFootLandingAngle(const Foot & current_moving_foot, const Eigen::Vector3d & landing);

  void updateFootSwingPose(mc_control::fsm::Controller & ctl, const Foot & current_moving_foot, const sva::PTransformd & X_0_landing);

  Foot getCurrentMovingFoot(mc_control::fsm::Controller & ctl)
  {
    Foot current_moving_foot;
    if(controller().supportContact().surfaceName == "LeftFootCenter")
    {
      current_moving_foot = Foot::Right;
    }
    else
    {
      current_moving_foot = Foot::Left;
    }
    return current_moving_foot;
  }

  bool evaluateTheta(double theta, double direction, const Eigen::Vector3d& pos, const std::vector<Eigen::Vector3d>& ground, Eigen::Vector2d& output);

protected:
  // Used to compute the cost to understand which is the best stiffness
  double lambda_zmp_ = 3.0;
  std::vector<double> zmp_;

  double lambda_CoM_ = 3.0;
  std::vector<double> CoM_;

  double nr_remaining_footstep_ = 0.0;
  double nr_footstep_ = 0.0;
  double lambda_footstep_ = 10.0;

  double cost_ = 0.0;
  double PhalangesStiffness_ = 0.0;
  double extra_to_compute_best_position_ = 0.01;

  // FootData contains data used to estimate the ground profile
  struct FootData
  {
    double range;
    std::vector<Eigen::Vector3d> ground;
    std::vector<std::array<double, 2>> phalanxes;
    std::vector<double> altitude;
    double k;
    double angle;
    double min_max_phalanxes_angle;
    double position_offset_x;
    double position_offset_z;
    bool need_reset;
    bool computation_done;
  };
  std::unordered_map<Foot, FootData> foot_data_;

  // Data for GroundSegment to compute best position/orientation
  struct GroundSegment
  {
    std::vector<Eigen::Vector3d> raw; // means all the data
    std::vector<Eigen::Vector3d> filtered;
    std::vector<Eigen::Vector3d> convex;
  };
  std::unordered_map<Foot, GroundSegment> ground_segment_;

  std::unordered_map<Foot, std::string> variable_stiffness_jointname_ = {
    {Foot::Left, "L_VARSTIFF"},
    {Foot::Right, "R_VARSTIFF"}
  };

  std::unordered_map<Foot, std::string> range_sensor_name_ = {
    {Foot::Left, "LeftFootRangeSensor"},
    {Foot::Right, "RightFootRangeSensor"}
  };

  std::unordered_map<Foot, std::string> surface_name_ = {
    {Foot::Left, "LeftFootCenter"},
    {Foot::Right, "RightFootCenter"}
  };

  // Keep preivous poses for ground estimation
  double delay_of_estimation_ = 0.005;
  std::unordered_map<Foot, Circular_Buffer<sva::PTransformd>> past_foot_pose_;

  // TODO: Ugly hardcoded value
  // double foot_length_ = 0.27742;
  double foot_length_ = 0.34742;
  double landing_to_foot_middle_offset_ = 0.0328;
  size_t nr_phalanxes_;
  double phalanx_length_;
  double time_ = 0.; // controller time

  // Reset data
  void reset(mc_control::fsm::Controller & ctl, const Foot & foot);

  // Client is here to call the service to compute the stiffness based on the ground profile
  ros::ServiceClient client_;

  // options
  bool with_variable_stiffness_ = false;
  bool with_ankle_rotation_ = false;
  bool with_foot_adjustment_ = false;

  // Subscriber to update the range sensor
  ros::Subscriber right_foot_range_sensor_sub_;
  void rightFRSCallback(const std_msgs::Float64::ConstPtr& data);

  ros::Subscriber left_foot_range_sensor_sub_;
  void leftFRSCallback(const std_msgs::Float64::ConstPtr& data);

  std::mutex range_sensor_mutex_;

};

} // namespace lipm_walking::states
