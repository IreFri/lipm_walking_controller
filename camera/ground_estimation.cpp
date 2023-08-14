/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <vector>
#include <string>
#include <exception>
#include <fstream>
#include <memory>
#include <thread>
#include <chrono>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/gui.h>
#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/logging.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rbdyn/Robots.h>

#include <mc_mujoco/devices/RangeSensor.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <open3d/Open3D.h>

namespace sva
{

using PTransformd = PTransform<double>;

}

namespace
{

/** We redefined sva::Rot. functions to make them work with non-scalar types */

template<typename T>
inline Eigen::Matrix3<T> RotX(T theta)
{
  T s = sin(theta), c = cos(theta);
  return (Eigen::Matrix3<T>() << T(1), T(0), T(0), T(0), c, s, T(0), -s, c).finished();
}

template<typename T>
inline Eigen::Matrix3<T> RotY(T theta)
{
  T s = sin(theta), c = cos(theta);
  return (Eigen::Matrix3<T>() << c, T(0), -s, T(0), T(1), T(0), s, T(0), c).finished();
}

template<typename T>
inline Eigen::Matrix3<T> RotZ(T theta)
{
  T s = sin(theta), c = cos(theta);
  return (Eigen::Matrix3<T>() << c, s, T(0), -s, c, T(0), T(0), T(0), T(1)).finished();
}

template<typename T>
Eigen::Matrix3<T> rpyToMat(const T & r, const T & p, const T & y)
{
  return RotX<T>(r) * RotY<T>(p) * RotZ<T>(y);
}

struct PitchZCostFunctor
{
  PitchZCostFunctor(const sva::PTransformd& X_s_p, const sva::PTransformd & X_0_b, const sva::PTransformd & X_b_s)
  : X_s_p_(X_s_p), X_0_b_(X_0_b), X_b_s_(X_b_s)
  {
  }

  template<typename T>
  bool operator()(const T * const pitch, const T * const t_z, T * residual)
      const
  {
    sva::PTransform<T> X_0_b = X_0_b_;
    sva::PTransform<T> X_b_b(RotY(pitch[0]), Eigen::Vector3<T>(T(0), T(0), t_z[0]));
    sva::PTransform<T> X_b_s = X_b_s_;
    sva::PTransform<T> X_s_p = X_s_p_;

    sva::PTransform<T> X_0_p = (X_s_p * X_b_s * X_b_b * X_0_b);

    residual[0] = X_0_p.translation().z();

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(const sva::PTransformd& X_s_p, const sva::PTransformd & X_0_b, const sva::PTransformd & X_b_s)
  {
    return (new ceres::AutoDiffCostFunction<PitchZCostFunctor, 1, 1, 1>(new PitchZCostFunctor(X_s_p, X_0_b, X_b_s)));
  }

private:
  const sva::PTransformd X_s_p_;
  const sva::PTransformd X_0_b_;
  const sva::PTransformd X_b_s_;
};

struct PitchCostFunctor
{
  PitchCostFunctor(const sva::PTransformd& X_s_p, const sva::PTransformd & X_0_b, const sva::PTransformd & X_b_s)
  : X_s_p_(X_s_p.cast<double>()), X_0_b_(X_0_b.cast<double>()), X_b_s_(X_b_s.cast<double>())
  {
  }

  template<typename T>
  bool operator()(const T * const pitch, T * residual)
      const
  {
    sva::PTransform<T> X_0_b = X_0_b_;
    sva::PTransform<T> X_b_b(RotY(pitch[0]));
    sva::PTransform<T> X_b_s = X_b_s_;
    sva::PTransform<T> X_s_p = X_s_p_;

    residual[0] = ceres::exp(- (X_s_p * X_b_s * X_b_b * X_0_b).translation().z() * (X_s_p * X_b_s * X_b_b * X_0_b).translation().z());

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(const sva::PTransformd& X_s_p, const sva::PTransformd & X_0_b, const sva::PTransformd & X_b_s)
  {
    return (new ceres::AutoDiffCostFunction<PitchCostFunctor, 1, 1>(new PitchCostFunctor(X_s_p, X_0_b, X_b_s)));
  }

private:
  const sva::PTransformd X_s_p_;
  const sva::PTransformd X_0_b_;
  const sva::PTransformd X_b_s_;
};

struct Minimize
{
  template<typename T>
  bool operator()(const T * const x, T * residual) const
  {
    residual[0] = x[0];
    return true;
  }
};


template<size_t polynomial_coeff_size>
struct CostFunctor
{
  CostFunctor(double x, double y)
  : x_(x),
    y_(y)
  {}

  template <typename T>
  bool operator()(const T* const polynomial_coeff, T* residuals) const
  {
    T _x = T(x_);
    T _y = T(y_);
    T sum_poly = polynomial_coeff[0];
    T x_pow = _x;
    for(size_t i = 1; i < polynomial_coeff_size; ++i)
    {
      sum_poly = sum_poly + (polynomial_coeff[i] * x_pow);
      x_pow = x_pow * _x;
    }
    residuals[0] = _y - sum_poly;
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(double x, double y)
   {
     return (new ceres::AutoDiffCostFunction<CostFunctor, 1, 4>(new CostFunctor(x, y)));
   }

private:
  const double x_;
  const double y_;
};

}

struct LogRobot
{
  struct Configuration
  {
    /** RobotModule used for robots initialization */
    mc_rbdyn::RobotModulePtr rm;
    /** Timestep of the log */
    double dt;
    /** Identifier and TF prefix */
    std::string id;
    /** Configuration entry in the log */
    std::string configuration;
    /** Encoders entry in the log */
    std::string encoders;
    /** PTransfomd entry used for doubleing-base estimation */
    std::string base;
    /** If not empty, use this as doubleing-base rotation */
    std::string base_rotation;
    /** If not empty, use this as doubleing-base translation */
    std::string base_translation;
    /** If true, interpret base_rotation as an IMU reading */
    bool base_rotation_is_imu = false;
  };

  /** Create a LogRobot with the provided configuration */
  LogRobot(const Configuration & config)
  : config_(config), robots_(mc_rbdyn::loadRobot(*config.rm))
  {
  }


  /** Update the published robot state */
  void update(const mc_rtc::log::FlatLog & log, size_t i)
  {
    q = log.get(config_.configuration, i, q);
    encoders = log.get(config_.encoders, i, encoders);
    if(config_.base.size())
    {
      base = log.get(config_.base, i, base);
    }
    if(config_.base_translation.size())
    {
      base.translation() = log.get(config_.base_translation, i, base.translation());
    }
    if(config_.base_rotation.size())
    {
      if(config_.base_rotation_is_imu)
      {
        if(i == 0)
        {
          X_imu0_0 = {log.get<Eigen::Quaterniond>(config_.base_rotation, i, {1, 0, 0, 0})};
          X_imu0_0 = X_imu0_0.inv();
        }
        sva::PTransformd X_0_imu = {log.get<Eigen::Quaterniond>(config_.base_rotation, i, {1, 0, 0, 0})};
        base.rotation() = (X_0_imu * X_imu0_0).rotation();
      }
      else
      {
        base.rotation() = Eigen::Matrix3d(log.get(config_.base_rotation, i, Eigen::Quaterniond(base.rotation())));
      }
    }
    auto & robot = robots_->robot();
    for(size_t refIdx = 0; refIdx < robot.refJointOrder().size(); ++refIdx)
    {
      const auto & jN = robot.refJointOrder()[refIdx];
      if(robot.hasJoint(jN))
      {
        auto jIndex = robot.jointIndexByName(jN);
        if(robot.mbc().q[jIndex].size() == 1)
        {
          robot.mbc().q[jIndex][0] = q[refIdx];
        }
      }
    }
    robot.posW(base);
  }

  /** Access underlying robot */
  inline const mc_rbdyn::Robot & robot() const
  {
    return robots_->robot();
  }

private:
  Configuration config_;
  std::shared_ptr<mc_rbdyn::Robots> robots_;
  sva::PTransformd base;
  std::vector<double> q;
  std::vector<double> encoders;
  // Inverse of initial IMU orientation
  sva::PTransformd X_imu0_0;
};


struct LogExplorer
{
public:
  LogExplorer(const std::string & logfile, mc_rbdyn::RobotModulePtr mod, double dt)
  : mod_(mod), dt_(dt)
  {
    log_.load(logfile);
    if(log_.has("t"))
    {
      const auto & t = log_.getRaw<double>("t");
      if(t.size() > 2 && t[0] && t[1])
      {
        dt_ = *t[1] - *t[0];
        mc_rtc::log::info("[LogExplorer] Log timestep: {}", dt_);
      }
    }

    LogRobot::Configuration conf;
    conf.rm = mod;
    conf.dt = dt_;
    conf.encoders = "qIn";

    // Only use control roobt
    // {
    //   conf.id = "control";
    //   conf.configuration = "qOut";
    //   conf.base = "ff";
    //   robot_.reset(new LogRobot(conf));
    // }
    // Only use real roobt
    {
      conf.id = "real";
      conf.configuration = "qIn";
      if(log_.has("realRobot_posW"))
      {
        conf.base = "realRobot_posW";
        robot_.reset(new LogRobot(conf));
      }
      else if(log_.has("rpyIn"))
      {
        conf.base = "ff";
        conf.base_rotation = "rpyIn";
        conf.base_rotation_is_imu = true;
        robot_.reset(new LogRobot(conf));
      }
      else
      {
        robot_.reset(nullptr);
      }
    }
  }

  bool setupTimeSection(const std::string & t)
  {
    size_t i = 0;
    double t_i = 0;
    double t_ii = 0;
    do
    {
      t_i = log_.get<double>(t, i, 0);
      t_ii = log_.get<double>(t, i + 1, 0);
    } while(t_i == t_ii && ++i < log_.size() - 1);
    if(i + 1 == log_.size())
    {
      return false;
    }
    if(fabs(t_ii - t_i - dt_) > 1e-6)
    {
      i = i + 1;
    }
    size_t start_i = i;
    do
    {
      t_i = log_.get<double>(t, i, 0);
      t_ii = log_.get<double>(t, i + 1, 0);
    } while(fabs(t_ii - t_i - dt_) < 1e-6 && ++i < log_.size() - 1);
    if(i == start_i)
    {
      return false;
    }
    min_i_ = start_i;
    cur_i_ = min_i_;
    max_i_ = i;
    min_t_ = log_.get<double>(t, min_i_, 0);
    cur_t_ = min_t_;
    max_t_ = log_.get<double>(t, max_i_, 0);
    t_data = t;

    return true;
  }

  bool run()
  {
    robot_->update(log_, cur_i_);

    if(cur_i_ >= max_i_)
    {
      return false;
    }
    if(cur_i_ < min_i_)
    {
      cur_i_ = max_i_;
    }
    cur_t_ = log_.get("t", cur_i_, cur_t_);

    ++ cur_i_;

    return true;
  }

  const size_t& cur_i() const
  {
    return cur_i_;
  }

  const mc_rtc::log::FlatLog& log() const
  {
    return log_;
  }

  inline const mc_rbdyn::Robot & robot() const
  {
    return robot_->robot();
  }

public:
  std::shared_ptr<mc_rbdyn::RobotModule> mod_;

  /** Log data */
  mc_rtc::log::FlatLog log_;

  /** Time parameters */
  double dt_;

  /* min/max/current playback index/time */
  double min_t_ = 0;
  size_t min_i_ = 0;
  double max_t_ = 0;
  size_t max_i_ = 0;
  double cur_t_ = 0;
  size_t cur_i_ = 0;

  /* Current time data */
  std::string t_data;

  std::unique_ptr<LogRobot> robot_;
};

struct FootDataHolder
{
public:
  // sva::PTransformd
  sva::PTransformd X_0_f = sva::PTransformd::Identity();
  sva::PTransformd X_0_s = sva::PTransformd::Identity();
  sva::PTransformd X_0_b = sva::PTransformd::Identity();
  sva::PTransformd X_b_s = sva::PTransformd::Identity();

  // Robot relative name
  std::string surface_name;
  std::string force_sensor_name;
  std::string camera_sensor_name;

  // Log relative name
  std::string path_to_points;

  // Data
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> icp_points;
  std::vector<Eigen::Vector3d> last_icp_points;
  std::vector<Eigen::Vector3d> previous_points;
  std::vector<Eigen::Vector3d> camera_points;
  std::vector<Eigen::Vector3d> corrected_points;

  std::vector<double> points_x;
  std::vector<double> points_y;
  std::vector<double> points_z;

  std::vector<double> previous_points_x;
  std::vector<double> previous_points_y;
  std::vector<double> previous_points_z;

  // For Open3D ICP
  std::shared_ptr<open3d::geometry::PointCloud> source;
  std::shared_ptr<open3d::geometry::PointCloud> source_transformed;
  std::shared_ptr<open3d::geometry::PointCloud> reference_source;


  // For Ceres
  double pitch = 0.;
  double t_z = 0.;

  std::vector<double> corrected_points_x;
  std::vector<double> corrected_points_y;
  std::vector<double> corrected_points_z;

  std::vector<double> every_foot_t_z;
  std::vector<double> every_pitch;
  std::vector<double> every_t_z;

  // For plotting
  std::vector<double> last_points_x;
  std::vector<double> last_points_y;
  std::vector<double> last_points_z;

  std::vector<double> last_icp_points_x;
  std::vector<double> last_icp_points_y;
  std::vector<double> last_icp_points_z;

  std::vector<double> avg_points_x;
  std::vector<double> avg_points_y;
  std::vector<double> avg_points_z;

  std::vector<double> foot_x;
  std::vector<double> foot_y;
  std::vector<double> foot_z;

  std::vector<double> icp_points_x;
  std::vector<double> icp_points_y;
  std::vector<double> icp_points_z;
};

int main(int argc, char * argv[])
{
  if(argc == 1 || argc > 3)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("It requires one or two arguments: /path/to/bin [optionnal] name of time section to use");
  }

  const std::string log_path(argv[1]);

  std::string time_section_name("t");
  if(argc == 3)
  {
    time_section_name = std::string(argv[2]);
  }

  mc_rbdyn::RobotModulePtr mod;
  {
    std::vector<std::string> robot_params = {"HRP4CR"};
    if(robot_params.size() == 1)
    {
      mod = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0]);
    }
    else if(robot_params.size() == 2)
    {
      mod = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0], robot_params[1]);
    }
    else if(robot_params.size() == 3)
    {
      mod = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0], robot_params[1], robot_params[2]);
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("log_visualization cannot handle the robot_params it was given");
    }
  }

  mc_rtc::log::info("Replaying log_: {}", log_path);
  double dt = 0.005f;

  // What do we want to compute
  FootDataHolder right_foot;
  right_foot.surface_name = "RightFootCenter";
  right_foot.camera_sensor_name = "RightFootCameraSensor";
  right_foot.force_sensor_name = "RightFootForceSensor";
  right_foot.path_to_points = "Observers_LIPMWalkingObserverPipeline_CameraRight_points";
  right_foot.source = std::make_shared<open3d::geometry::PointCloud>();
  right_foot.source_transformed = std::make_shared<open3d::geometry::PointCloud>();
  right_foot.reference_source = std::make_shared<open3d::geometry::PointCloud>();

  FootDataHolder left_foot;
  left_foot.surface_name = "LeftFootCenter";
  left_foot.camera_sensor_name = "LeftFootCameraSensor";
  left_foot.force_sensor_name = "LeftFootForceSensor";
  left_foot.path_to_points = "Observers_LIPMWalkingObserverPipeline_CameraLeft_points";
  left_foot.source = std::make_shared<open3d::geometry::PointCloud>();
  left_foot.source_transformed = std::make_shared<open3d::geometry::PointCloud>();
  left_foot.reference_source = std::make_shared<open3d::geometry::PointCloud>();

  // Replay
  LogExplorer appli(log_path, mod, dt);

  // FUNCTIONS
  auto updateDataForPlot = [](FootDataHolder& foot_holder)
  {
    double x = std::accumulate(foot_holder.points.begin(), foot_holder.points.end(), 0.f, [](double a, const Eigen::Vector3d& v) { return a + v.x(); });
    double y = std::accumulate(foot_holder.points.begin(), foot_holder.points.end(), 0.f, [](double a, const Eigen::Vector3d& v) { return a + v.y(); });
    double z = std::accumulate(foot_holder.points.begin(), foot_holder.points.end(), 0.f, [](double a, const Eigen::Vector3d& v) { return a + v.z(); });

    foot_holder.avg_points_x.push_back(x / static_cast<double>(foot_holder.points.size()));
    foot_holder.avg_points_y.push_back(y / static_cast<double>(foot_holder.points.size()));
    foot_holder.avg_points_z.push_back(z / static_cast<double>(foot_holder.points.size()));

    std::transform(foot_holder.camera_points.begin(), foot_holder.camera_points.end(), std::back_inserter(foot_holder.corrected_points_x), [](const Eigen::Vector3d & p) { return p.x(); });
    std::transform(foot_holder.camera_points.begin(), foot_holder.camera_points.end(), std::back_inserter(foot_holder.corrected_points_y), [](const Eigen::Vector3d & p) { return p.y(); });
    std::transform(foot_holder.camera_points.begin(), foot_holder.camera_points.end(), std::back_inserter(foot_holder.corrected_points_z), [](const Eigen::Vector3d & p) { return p.z(); });

    std::transform(foot_holder.points.begin(), foot_holder.points.end(), std::back_inserter(foot_holder.points_x), [](const Eigen::Vector3d & p) { return p.x(); });
    std::transform(foot_holder.points.begin(), foot_holder.points.end(), std::back_inserter(foot_holder.points_y), [](const Eigen::Vector3d & p) { return p.y(); });
    std::transform(foot_holder.points.begin(), foot_holder.points.end(), std::back_inserter(foot_holder.points_z), [](const Eigen::Vector3d & p) { return p.z(); });

    foot_holder.last_points_x.clear();
    foot_holder.last_points_y.clear();
    foot_holder.last_points_z.clear();
    std::transform(foot_holder.points.begin(), foot_holder.points.end(), std::back_inserter(foot_holder.last_points_x), [](const Eigen::Vector3d & p) { return p.x(); });
    std::transform(foot_holder.points.begin(), foot_holder.points.end(), std::back_inserter(foot_holder.last_points_y), [](const Eigen::Vector3d & p) { return p.y(); });
    std::transform(foot_holder.points.begin(), foot_holder.points.end(), std::back_inserter(foot_holder.last_points_z), [](const Eigen::Vector3d & p) { return p.z(); });

    foot_holder.last_icp_points_x.clear();
    foot_holder.last_icp_points_y.clear();
    foot_holder.last_icp_points_z.clear();
    std::transform(foot_holder.last_icp_points.begin(), foot_holder.last_icp_points.end(), std::back_inserter(foot_holder.last_icp_points_x), [](const Eigen::Vector3d & p) { return p.x(); });
    std::transform(foot_holder.last_icp_points.begin(), foot_holder.last_icp_points.end(), std::back_inserter(foot_holder.last_icp_points_y), [](const Eigen::Vector3d & p) { return p.y(); });
    std::transform(foot_holder.last_icp_points.begin(), foot_holder.last_icp_points.end(), std::back_inserter(foot_holder.last_icp_points_z), [](const Eigen::Vector3d & p) { return p.z(); });

    std::transform(foot_holder.icp_points.begin(), foot_holder.icp_points.end(), std::back_inserter(foot_holder.icp_points_x), [](const Eigen::Vector3d & p) { return p.x(); });
    std::transform(foot_holder.icp_points.begin(), foot_holder.icp_points.end(), std::back_inserter(foot_holder.icp_points_y), [](const Eigen::Vector3d & p) { return p.y(); });
    std::transform(foot_holder.icp_points.begin(), foot_holder.icp_points.end(), std::back_inserter(foot_holder.icp_points_z), [](const Eigen::Vector3d & p) { return p.z(); });

    foot_holder.foot_x.push_back(foot_holder.X_0_b.translation().x());
    foot_holder.foot_y.push_back(foot_holder.X_0_b.translation().y());
    foot_holder.foot_z.push_back(foot_holder.X_0_b.translation().z());
  };

  auto extract = [&appli](FootDataHolder& foot_holder, size_t cur_i_offset = 0) -> void
  {

    const auto& log = appli.log();
    const auto& cur_i = appli.cur_i();
    appli.robot_->update(log, cur_i - cur_i_offset);
    const auto& robot = appli.robot();

    foot_holder.X_0_f = robot.surfacePose(foot_holder.surface_name);

    std::vector<double> _points_x = log.get<std::vector<double>>(foot_holder.path_to_points + "_x", cur_i, {});
    std::vector<double> _points_y = log.get<std::vector<double>>(foot_holder.path_to_points + "_y", cur_i, {});
    std::vector<double> _points_z = log.get<std::vector<double>>(foot_holder.path_to_points + "_z", cur_i, {});

    std::vector<double> points_x;
    std::transform(_points_x.begin(), _points_x.end(), std::back_inserter(points_x), [](double v) { return static_cast<double>(v); });

    std::vector<double> points_y;

    std::transform(_points_y.begin(), _points_y.end(), std::back_inserter(points_y), [](double v) { return static_cast<double>(v); });
    std::vector<double> points_z;

    std::transform(_points_z.begin(), _points_z.end(), std::back_inserter(points_z), [](double v) { return static_cast<double>(v); });


    // Initialize previous points
    if(foot_holder.previous_points_x.empty() || points_x.empty())
    {
      foot_holder.previous_points_x = points_x;
      foot_holder.previous_points_y = points_y;
      foot_holder.previous_points_z = points_z;
      return;
    }

    // Check if new data
    if(foot_holder.previous_points_x.front() != points_x.front())
    {
      foot_holder.previous_points_x = points_x;
      foot_holder.previous_points_y = points_y;
      foot_holder.previous_points_z = points_z;

      const std::string& body_of_sensor = robot.device<mc_mujoco::RangeSensor>(foot_holder.camera_sensor_name).parent();
      // Access the position of body name in world coordinates (balanx position)
      foot_holder.X_0_b = robot.bodyPosW(body_of_sensor);
      // Returns the transformation from the parent body to the sensor
      foot_holder.X_b_s = robot.device<mc_mujoco::RangeSensor>(foot_holder.camera_sensor_name).X_p_s();

      // Keep the sensor pose when we receive a new set of data
      foot_holder.X_0_s = foot_holder.X_b_s * foot_holder.X_0_b;
    }
    else
    {
      if(!foot_holder.points.empty())
      {
        foot_holder.previous_points = foot_holder.points;
      }
      foot_holder.points.clear();
      return;
    }

    foot_holder.points.clear();
    foot_holder.camera_points.clear();
    // Compute the 3D point in world frame
    for(size_t i = 0; i < points_x.size(); ++i)
    {
      Eigen::Vector3d pos(points_x[i], points_y[i], points_z[i]);
      // From camera frame to world frame
      const sva::PTransformd X_s_p(pos);
      const sva::PTransformd X_0_p = X_s_p * foot_holder.X_0_s;
      if(X_s_p.translation().x() < 0.05 && X_s_p.translation().z() < 0.30)
      {
        foot_holder.camera_points.push_back(pos);
        foot_holder.points.push_back(X_0_p.translation());
      }
    }

    appli.robot_->update(log, cur_i);
  };

  open3d::visualization::Visualizer visualizer;
  if(!visualizer.CreateVisualizerWindow("Ground Estimation", 640, 480, 50, 50))
  {
    open3d::utility::LogWarning(
                "[DrawGeometries] Failed creating OpenGL "
                "window.");
        return false;
  }

  visualizer.GetRenderOption().point_show_normal_ = false;
  visualizer.GetRenderOption().mesh_show_wireframe_ = false;
  visualizer.GetRenderOption().mesh_show_back_face_ = false;

  // Open3D Rendering Loop
  visualizer.BuildUtilities();
  visualizer.UpdateWindowTitle();

  // Offset for viewing
  Eigen::Matrix4d plus_offset_view = Eigen::Matrix4d::Identity();
  plus_offset_view.block<3, 1>(0, 3) = Eigen::Vector3d(0., 0.005, 0.);


  appli.setupTimeSection(time_section_name);

  size_t counter = 0;
  while(true)
  {
    if(!appli.run())
    {
      break;
    }

    const auto& log = appli.log();
    const auto& cur_i = appli.cur_i();

    const std::string state = log.get<std::string>("Executor_LIPMWalking::WalkInternal", cur_i, "None");

    if(state == "None")
    {
      continue;
    }

    if(state == "LIPMWalking::Standing")
    {
      // break;
      continue;
    }

    if(state == "LIPMWalking::DoubleSupport")
    {
      // break;
      continue;
    }

    auto _foot_holder = std::reference_wrapper<FootDataHolder>{left_foot};
    // for(auto& _foot_holder: std::vector<std::reference_wrapper<FootDataHolder>>{right_foot})
    // for(auto& _foot_holder: std::vector<std::reference_wrapper<FootDataHolder>>{right_foot, left_foot})
    {
      auto& foot_holder = _foot_holder.get();

      const double fz = log.get<sva::ForceVecd>(foot_holder.force_sensor_name, cur_i, sva::ForceVecd{}).force().z();
      if(state == "LIPMWalking::SingleSupport" && fz > 20.)
      // if(state == "LIPMWalking::SingleSupport")
      {
        continue;
      }

      // 1- Get the point as X_0_p
      extract(foot_holder, 0);

      if(foot_holder.points.empty())
      {
        continue;
      }

      mc_rtc::log::info("[Step 0] All");
      auto start = std::chrono::high_resolution_clock::now();
      {
        mc_rtc::log::info("[Step 1] Sort points alongside x");
        auto start = std::chrono::high_resolution_clock::now();
        std::sort(foot_holder.points.begin(), foot_holder.points.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return a.x() < b.x(); });
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        mc_rtc::log::warning("[Step 1] It tooks {} microseconds -> {} milliseconds", duration.count(), duration.count() / 1000.);
      }

      {
        mc_rtc::log::info("[Step 2] Estimate the z - pitch to bring back to 0");
        auto start = std::chrono::high_resolution_clock::now();
        foot_holder.pitch = 0.;
        foot_holder.t_z = 0.;

        ceres::Problem problem;
        for(const auto& point: foot_holder.camera_points)
        {
          ceres::CostFunction* cost_function = PitchZCostFunctor::Create(sva::PTransformd(point), foot_holder.X_0_b, foot_holder.X_b_s);
          problem.AddResidualBlock(cost_function,  new ceres::CauchyLoss(0.5), &foot_holder.pitch, &foot_holder.t_z);
        }
        ceres::CostFunction * min_p = new ceres::AutoDiffCostFunction<Minimize, 1, 1>(new Minimize());
        problem.AddResidualBlock(min_p, nullptr, &foot_holder.pitch);
        ceres::CostFunction * min_z = new ceres::AutoDiffCostFunction<Minimize, 1, 1>(new Minimize());
        problem.AddResidualBlock(min_z, nullptr, &foot_holder.t_z);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        const sva::PTransformd X_0_b = foot_holder.X_0_b;
        const sva::PTransformd X_b_b(sva::RotY(foot_holder.pitch).cast<double>(), Eigen::Vector3d(0., 0., foot_holder.t_z));
        const sva::PTransformd X_b_s = foot_holder.X_b_s;

        // Compute the 3D point in world frame
        foot_holder.corrected_points.clear();
        for(const auto& point: foot_holder.camera_points)
        {
          // From camera frame to world frame
          const sva::PTransformd X_s_p(point);
          const sva::PTransformd X_0_p = (X_s_p * X_b_s * X_b_b * X_0_b);
          if(X_s_p.translation().x() < 0.05)
          {
            foot_holder.corrected_points.push_back(X_0_p.translation());
          }
        }
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        mc_rtc::log::warning("[Step 2] It tooks {} microseconds -> {} milliseconds", duration.count(), duration.count() / 1000.);
      }

      {
        mc_rtc::log::info("[Step 3] Perform ICP");
        auto start = std::chrono::high_resolution_clock::now();
        foot_holder.source->points_ = foot_holder.points;

        if(foot_holder.reference_source->points_.empty())
        {
          *foot_holder.reference_source = *foot_holder.source;
          continue;
        }

        foot_holder.source = foot_holder.source->VoxelDownSample(0.005);
        // TODO: This depend on the foot pose
        foot_holder.reference_source = foot_holder.reference_source->Crop(open3d::geometry::AxisAlignedBoundingBox(
          Eigen::Vector3d(0.10, -0.5, -0.04), Eigen::Vector3d(0.50, 0.5, 0.15)));
        foot_holder.reference_source = foot_holder.reference_source->VoxelDownSample(0.01);

        // ICP For matching
        auto result = open3d::pipelines::registration::RegistrationGeneralizedICP(
            *foot_holder.source, *foot_holder.reference_source, 0.05, Eigen::Matrix4d::Identity(),
            open3d::pipelines::registration::TransformationEstimationForGeneralizedICP(),
            open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 15));

        // Apply transformation
        *foot_holder.source_transformed = *foot_holder.source;
        foot_holder.source_transformed->Transform(result.transformation_);

        *foot_holder.reference_source += *foot_holder.source_transformed;
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        mc_rtc::log::warning("[Step 3] It tooks {} microseconds -> {} milliseconds", duration.count(), duration.count() / 1000.);

        std::shared_ptr<open3d::geometry::PointCloud> view(new open3d::geometry::PointCloud);
        std::shared_ptr<open3d::geometry::PointCloud> view_source(new open3d::geometry::PointCloud);
        *view = *foot_holder.reference_source;
        *view_source = *foot_holder.source;
        for(size_t o = 0; o < counter * 2 + 4; ++o)
        {
          view_source->Transform(plus_offset_view);
          view->Transform(plus_offset_view);
        }
        view->Transform(plus_offset_view);
        view->PaintUniformColor(Eigen::Vector3d(0., 0., 1.));
        view_source->PaintUniformColor(Eigen::Vector3d(1., 0., 0.));
        visualizer.AddGeometry(view);
        visualizer.AddGeometry(view_source);

        ++ counter;
      }

      auto stop = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
      mc_rtc::log::warning("[Step 0] It tooks {} microseconds -> {} milliseconds", duration.count(), duration.count() / 1000.);


      // For plotting
      updateDataForPlot(foot_holder);
    }
  }

  mc_rtc::log::info("Done !");

  visualizer.AddGeometry(left_foot.reference_source);

  while(visualizer.PollEvents())
  {
    visualizer.UpdateGeometry();
    // Set render flag as dirty anyways, because when we use callback
    // functions, we assume something has been changed in the callback
    // and the redraw event should be triggered.
    visualizer.UpdateRender();
  }

  return 0;
}
//