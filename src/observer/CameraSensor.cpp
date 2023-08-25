#include <lipm_walking/observer/CameraSensor.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/version.h>
#include <mc_control/MCController.h>
#include <mc_mujoco/devices/RangeSensor.h>
#include <chrono>
#include <fstream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>


#include <ceres/ceres.h>
#include <ceres/rotation.h>

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

struct Minimize
{
  template<typename T>
  bool operator()(const T * const x, T * residual) const
  {
    residual[0] = x[0];
    return true;
  }
};

}

namespace lipm_walking
{

CameraSensor::CameraSensor(const std::string & type, double dt)
: mc_observers::Observer(type, dt)
{
}

void CameraSensor::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_name_ = config("robot", ctl.robot().name());

  if(config.has("camera_serial"))
  {
    // camera_serial_ = static_cast<std::string>(config("camera_serial"));
    camera_serial_ = std::to_string(static_cast<unsigned long int>(static_cast<double>(config("camera_serial"))));
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[CameraSensor::{}] 'camera_serial' is mandatory in the configuration.", name_);
  }

  if(config.has("sensor_name"))
  {
    sensor_name_ = static_cast<std::string>(config("sensor_name"));
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[CameraSensor::{}] 'sensor_name' is mandatory in the configuration.", name_);
  }

  if(config.has("desired_state"))
  {
    // camera_serial_ = static_cast<std::string>(config("camera_serial"));
    desired_state_ = static_cast<std::string>(config("desired_state"));
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[CameraSensor::{}] 'desired_state' is mandatory in the configuration; it can be 'LeftSwing' or 'RightSwing'", name_);
  }

  if(config.has("path_to_preset"))
  {
    path_to_preset_ = static_cast<std::string>(config("path_to_preset"));
  }

  if(config.has("kernel_size"))
  {
    kernel_size_ = config("kernel_size");
    kernel_size_ = kernel_size_ < 3 ? 3 : kernel_size_;
  }

  if(config.has("kernel_threshold"))
  {
    kernel_threshold_ = static_cast<float>(static_cast<double>(config("kernel_threshold")));
    kernel_threshold_ = kernel_threshold_ < 0 ? 0.005f : kernel_threshold_;
  }

  if(config.has("outlier_threshold"))
  {
    outlier_threshold_ = static_cast<float>(static_cast<double>(config("outlier_threshold")));
    outlier_threshold_ = outlier_threshold_ < 0 ? 0.01f : outlier_threshold_;
  }

  if(config.has("path_to_replay_data"))
  {
    path_to_replay_data_ = static_cast<std::string>(config("path_to_replay_data"));
  }

  mc_rtc::log::info("[CameraSensor::{}] 'sensor_name' is {}", name_, sensor_name_);
  mc_rtc::log::info("[CameraSensor::{}] 'camera_serial' is {}", name_, camera_serial_);
  mc_rtc::log::info("[CameraSensor::{}] 'path_to_preset' is {}", name_, path_to_preset_);
  mc_rtc::log::info("[CameraSensor::{}] 'kernel_size' is {}", name_, kernel_size_);
  mc_rtc::log::info("[CameraSensor::{}] 'kernel_threshold' is {}", name_, kernel_threshold_);
  mc_rtc::log::info("[CameraSensor::{}] 'outlier_threshold' is {}", name_, outlier_threshold_);

  startReadingCamera();

  desc_ = fmt::format("{} (sensor_name={}, camera_serial={}, kernel_size={}, kernel_threshold={}, outlier_threshold={})", name_, sensor_name_, camera_serial_, kernel_size_, kernel_threshold_, outlier_threshold_);
}

void CameraSensor::reset(const mc_control::MCController & ctl)
{
  if(!estimation_loop_.joinable())
  {
    mc_rtc::log::info("[CameraSensor::{}] Starting estimation thread", name_);
    startGroundEstimation(const_cast<mc_control::MCController &>(ctl));
  }
  // Nothing to do here
}

bool CameraSensor::run(const mc_control::MCController &)
{
  // Nothing to do here

  return true;
}

void CameraSensor::update(mc_control::MCController & ctl)
{
  t_ += ctl.solver().dt();
  if(new_ground_data_.load())
  {
    const std::lock_guard<std::mutex> lock(estimation_mutex_);
    if(pc_full_ground_reconstructed_points_)
    {
      if(!pc_full_ground_reconstructed_points_->points_.empty())
      {
        ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).update(pc_full_ground_reconstructed_points_->points_, 0.);
      }
    }
    new_ground_data_ = false;
  }
}

void CameraSensor::addToLogger(const mc_control::MCController &,
                                   mc_rtc::Logger & logger,
                                   const std::string & category)
{
  logger.addLogEntry(category + "_range", [this]() -> double
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return points_.empty() ? 0. : points_[0].z();
  });
  logger.addLogEntry(category + "_camera_points_x", [this]() -> std::vector<double>
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::vector<double> d;
    std::transform(points_.begin(), points_.end(), std::back_inserter(d),
      [](const Eigen::Vector3d & v) { return v.x(); });
    return d;
  });
  logger.addLogEntry(category + "_camera_points_y", [this]() -> std::vector<double>
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::vector<double> d;
    std::transform(points_.begin(), points_.end(), std::back_inserter(d),
      [](const Eigen::Vector3d & v) { return v.y(); });
    return d;
  });
  logger.addLogEntry(category + "_camera_points_z", [this]() -> std::vector<double>
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::vector<double> d;
    std::transform(points_.begin(), points_.end(), std::back_inserter(d),
      [](const Eigen::Vector3d & v) { return v.z(); });
    return d;
  });

  logger.addLogEntry(category + "_world_points_x", [this]() -> std::vector<double>
  {
    const std::lock_guard<std::mutex> lock(estimation_mutex_);
    std::vector<double> d;
    std::transform(ground_points_.begin(), ground_points_.end(), std::back_inserter(d),
      [](const Eigen::Vector3d & v) { return v.x(); });
    return d;
  });
  logger.addLogEntry(category + "_world_points_y", [this]() -> std::vector<double>
  {
    const std::lock_guard<std::mutex> lock(estimation_mutex_);
    std::vector<double> d;
    std::transform(ground_points_.begin(), ground_points_.end(), std::back_inserter(d),
      [](const Eigen::Vector3d & v) { return v.y(); });
    return d;
  });
  logger.addLogEntry(category + "_world_points_z", [this]() -> std::vector<double>
  {
    const std::lock_guard<std::mutex> lock(estimation_mutex_);
    std::vector<double> d;
    std::transform(ground_points_.begin(), ground_points_.end(), std::back_inserter(d),
      [](const Eigen::Vector3d & v) { return v.z(); });
    return d;
  });


  logger.addLogEntry(category + "_world_transformed_points_x", [this]() -> std::vector<double>
  {
    const std::lock_guard<std::mutex> lock(estimation_mutex_);
    std::vector<double> d;
    if(pc_transformed_estimated_ground_points_)
    {
      std::transform(pc_transformed_estimated_ground_points_->points_.begin(), pc_transformed_estimated_ground_points_->points_.end(), std::back_inserter(d),
        [](const Eigen::Vector3d & v) { return v.x(); });
    }
    return d;
  });
  logger.addLogEntry(category + "_world_transformed_points_y", [this]() -> std::vector<double>
  {
    const std::lock_guard<std::mutex> lock(estimation_mutex_);
    std::vector<double> d;
    if(pc_transformed_estimated_ground_points_)
    {
      std::transform(pc_transformed_estimated_ground_points_->points_.begin(), pc_transformed_estimated_ground_points_->points_.end(), std::back_inserter(d),
        [](const Eigen::Vector3d & v) { return v.y(); });
    }
    return d;
  });
  logger.addLogEntry(category + "_world_transformed_points_z", [this]() -> std::vector<double>
  {
    const std::lock_guard<std::mutex> lock(estimation_mutex_);
    std::vector<double> d;
    if(pc_transformed_estimated_ground_points_)
    {
      std::transform(pc_transformed_estimated_ground_points_->points_.begin(), pc_transformed_estimated_ground_points_->points_.end(), std::back_inserter(d),
        [](const Eigen::Vector3d & v) { return v.z(); });
    }
    return d;
  });

  logger.addLogEntry(category + "_ground_points_x", [this]() -> std::vector<double>
  {
    const std::lock_guard<std::mutex> lock(estimation_mutex_);
    std::vector<double> d;
    if(pc_full_ground_reconstructed_points_)
    {
      std::transform(pc_full_ground_reconstructed_points_->points_.begin(), pc_full_ground_reconstructed_points_->points_.end(), std::back_inserter(d),
        [](const Eigen::Vector3d & v) { return v.x(); });
    }
    return d;
  });
  logger.addLogEntry(category + "_ground_points_y", [this]() -> std::vector<double>
  {
    const std::lock_guard<std::mutex> lock(estimation_mutex_);
    std::vector<double> d;
    if(pc_full_ground_reconstructed_points_)
    {
      std::transform(pc_full_ground_reconstructed_points_->points_.begin(), pc_full_ground_reconstructed_points_->points_.end(), std::back_inserter(d),
        [](const Eigen::Vector3d & v) { return v.y(); });
    }
    return d;
  });
  logger.addLogEntry(category + "_ground_points_z", [this]() -> std::vector<double>
  {
    const std::lock_guard<std::mutex> lock(estimation_mutex_);
    std::vector<double> d;
    if(pc_full_ground_reconstructed_points_)
    {
      std::transform(pc_full_ground_reconstructed_points_->points_.begin(), pc_full_ground_reconstructed_points_->points_.end(), std::back_inserter(d),
        [](const Eigen::Vector3d & v) { return v.z(); });
    }
    return d;
  });
}

void CameraSensor::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_range");
  logger.removeLogEntry(category + "_points_x");
  logger.removeLogEntry(category + "_points_y");
  logger.removeLogEntry(category + "_points_z");
}

void CameraSensor::addToGUI(const mc_control::MCController & ctl,
                                mc_rtc::gui::StateBuilder & gui,
                                const std::vector<std::string> & category)
{
  gui.addElement(category,
    mc_rtc::gui::Label("Frame acquistion [ms]",
      [this]()
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        return camera_duration_;
      }),
    mc_rtc::gui::Label("Ground reconstruction [ms]",
      [this]()
      {
        const std::lock_guard<std::mutex> lock(estimation_mutex_);
        return ground_reconstruction_duration_;
      }),
    mc_rtc::gui::Label("Range [m]",
      [this]()
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        return points_.empty() ? 0. : points_[0].z();
      }),
    mc_rtc::gui::Point3D("Point [3D]",
      [this]()
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        return points_.empty() ? Eigen::Vector3d::Zero() : points_[0];
      }),
    mc_rtc::gui::Label("nr points",
      [this]()
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        return points_.size();
      }),
    mc_rtc::gui::Transform(fmt::format("X_0_{}", name_),
      [this, &ctl]()
      {
        const std::string & body_name = ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name_).parent();
        const sva::PTransformd X_0_p = ctl.realRobot().bodyPosW(body_name);
        return ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s() * X_0_p;
      }),
    mc_rtc::gui::ArrayInput("Rotation [deg]", {"r", "p", "y"},
      [this, &ctl]() -> Eigen::Vector3d
      {
        return mc_rbdyn::rpyFromMat(ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s().rotation()).unaryExpr([](double x){return x * 180. / M_PI;});
      },
      [this, &ctl](const Eigen::Vector3d & new_rpy)
      {
        const sva::PTransformd current_X_p_s = ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s();
        const sva::PTransformd new_X_p_s(mc_rbdyn::rpyToMat(new_rpy.unaryExpr([](double x){return x * M_PI / 180.;})), current_X_p_s.translation());
        const_cast<mc_control::MCController &>(ctl).robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s(new_X_p_s);
      }),
    mc_rtc::gui::ArrayInput("Translation", {"x", "y", "z"},
      [this, &ctl]()
      {
        return ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s().translation();
      },
      [this, &ctl](const Eigen::Vector3d & new_translation)
      {
        const sva::PTransformd current_X_p_s = ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s();
        const sva::PTransformd new_X_p_s(current_X_p_s.rotation(), new_translation);
        const_cast<mc_control::MCController &>(ctl).robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s(new_X_p_s);
      }),
    mc_rtc::gui::Transform("X_p_s",
      [this, &ctl]()
      {
        const std::string & body_name = ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name_).parent();
        const sva::PTransformd X_0_p = ctl.realRobot().bodyPosW(body_name);
        return ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s() * X_0_p;
      },
      [this, &ctl](const sva::PTransformd & X_0_s)
      {
        const std::string & body_name = ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name_).parent();
        const sva::PTransformd X_0_p = ctl.realRobot().bodyPosW(body_name);
        const sva::PTransformd new_X_p_s = X_0_s * X_0_p.inv();
        const_cast<mc_control::MCController &>(ctl).robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s(new_X_p_s);
      })
  );

  std::vector<std::string> points_category = category;
  points_category.push_back("points");
  gui.addElement(points_category,
    mc_rtc::gui::Trajectory("Ground points", mc_rtc::gui::Color::Green,
      [this, &ctl]()
      {
        std::vector<Eigen::Vector3d> points;
        {
          const std::lock_guard<std::mutex> lock(mutex_);
          points = ground_points_;
        }

        if(points.empty())
        {
          return std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
        }

        return points;
      })
  );

  gui.addElement(points_category,
    mc_rtc::gui::Trajectory("Ground reconstructed",
      [this, &ctl]()
      {
        std::vector<Eigen::Vector3d> points;
        {
          const std::lock_guard<std::mutex> lock(estimation_mutex_);
          if(pc_full_ground_reconstructed_points_)
          {
            points = pc_full_ground_reconstructed_points_->points_;
          }
        }

        if(points.empty())
        {
          return std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
        }

        return points;
      })
  );

  gui.addPlot(
    fmt::format("CameraSensor::{}", name_),
    mc_rtc::gui::plot::X("t", [this, &ctl]() { return t_; }),
    mc_rtc::gui::plot::Y("range", [this]()
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        return points_.empty() ? 0. : points_[0].z();
      }, mc_rtc::gui::Color::Red)
  );
}

void CameraSensor::startReadingCamera()
{
  if(loop_sensor_.joinable())
  {
    loop_sensor_.join();
  }

  loop_sensor_ = std::thread(
    [this]()
    {
      rs2::config cfg;
      // Check if we read from a .bag or from a camera
      if(path_to_replay_data_ != "")
      {
        mc_rtc::log::info("[CameraSensor] We will use the recorded depth at {}", path_to_replay_data_);
        cfg.enable_device_from_file(path_to_replay_data_);
      }
      else
      {
        cfg.enable_device(camera_serial_);
      }

      // Declare RealSense pipeline, encapsulating the actual device and sensors
      rs2::pipeline pipe;
      //
      auto prof = pipe.start(cfg);

      // Load preset
      if(!path_to_preset_.empty())
      {
        std::ifstream t(path_to_preset_);
        std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
        rs2::device dev = prof.get_device();
        auto advanced = dev.as<rs400::advanced_mode>();
        advanced.load_json(str);
      }

      auto depthToPoint = [](const rs2_intrinsics& intrinsics, const std::array<float, 2>& pixel, float depth) -> Eigen::Vector3d
      {
        Eigen::Vector3f point;
        rs2_deproject_pixel_to_point(point.data(), &intrinsics, pixel.data(), depth);
        return point.cast<double>();
      };

      auto applyKernel = [this](const std::array<float, 2>& pixel, const rs2::depth_frame& frame) -> float
      {
        const float pixel_depth = frame.get_distance(static_cast<int>(pixel[0]), static_cast<int>(pixel[1]));
        float sum_depth = 0.f;
        float counter = 0.f;
        const int half_kernel_size = static_cast<int>(kernel_size_/2);
        for(int i = -half_kernel_size; i < half_kernel_size; ++i)
        {
          for(int j = -half_kernel_size; j < half_kernel_size; ++j)
          {
            const std::array<float, 2> ppixel = {pixel[0] + static_cast<float>(i), pixel[1] + static_cast<float>(j)};
            const float ddepth = frame.get_distance(static_cast<int>(ppixel[0]), static_cast<int>(ppixel[1]));
            if(std::abs(ddepth - pixel_depth) < kernel_threshold_)
            {
              sum_depth += ddepth;
              counter += 1.f;
            }
          }
        }
        return sum_depth / counter;
      };

      auto generateStatistics = [](double& mean, double& variance, double& stddev, const std::vector<double>& distances)
      {
        // Estimate the mean and the standard deviation of the distance vector
        double sum = 0, sq_sum = 0;
        for (const float &distance : distances)
        {
          sum += distance;
          sq_sum += distance * distance;
        }

        mean = sum / static_cast<double>(distances.size());
        variance = (sq_sum - sum * sum / static_cast<double>(distances.size())) / (static_cast<double>(distances.size()) - 1);
        stddev = sqrt (variance);
      };


      // Decimation filter reduces the amount of data (while preserving best samples)
      rs2::decimation_filter dec;
      // If the demo is too slow, make sure you run in Release (-DCMAKE_BUILD_TYPE=Release)
      // but you can also increase the following parameter to decimate depth more (reducing quality)
      dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 4);
      // HDR Merge
      rs2::hdr_merge hdr;
      // Threshold filter
      rs2::threshold_filter thresh(0.5f, 0.40f);
      // Define transformations from and to Disparity domain
      rs2::disparity_transform depth2disparity;
      // Define spatial filter (edge-preserving)
      rs2::spatial_filter spat;
      spat.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.f);
      spat.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
      spat.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.f);
      spat.set_option(RS2_OPTION_HOLES_FILL, 0);
      rs2::disparity_transform disparity2depth(false);

      while(!stop_loop_.load())
      {
        auto frames = pipe.wait_for_frames();
        auto start = std::chrono::high_resolution_clock::now();

        // Get depth frame
        auto frame = frames.get_depth_frame();
        frame = frame.apply_filter(dec);
        frame = frame.apply_filter(hdr);
        frame = frame.apply_filter(thresh);
        frame = frame.apply_filter(depth2disparity);
        frame = frame.apply_filter(spat);
        frame = frame.apply_filter(disparity2depth);

        const rs2_intrinsics intrinsics = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

        const size_t height = frame.get_height();
        const size_t width = frame.get_width();

        const size_t half_height = static_cast<size_t>(static_cast<double>(height) * 0.5);
        const size_t half_width = static_cast<size_t>(static_cast<double>(width) * 0.5);

        std::array<float, 2> pixel;
        const int half_kernel_size = static_cast<int>(kernel_size_/2);
        std::vector<Eigen::Vector3d> points;
        const int offset = 20;
        for(size_t i = half_height + half_kernel_size; i < height - half_kernel_size - offset; ++i)
        {
          pixel[0] = static_cast<float>(i);
          pixel[1] = static_cast<float>(half_height);

          const float depth = applyKernel(pixel, frame);
          Eigen::Vector3d point = depthToPoint(intrinsics, pixel, depth);
          if(point.z() != 0 && point.x() < 0.05 && point.z() < 0.30)
          {
            point.y() = 0.;
            points.push_back(point);
          }
        }

        // Sort
        std::sort(points.begin(), points.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return a.x() < b.x(); });

        {
          const std::lock_guard<std::mutex> lock(mutex_);
          points_ = points;
        }

        // Needs to update
        new_camera_data_ = true;

        auto stop = std::chrono::high_resolution_clock::now();
        {
          const std::lock_guard<std::mutex> lock(mutex_);
          camera_duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
        }
      }
    }
  );
}

void CameraSensor::startGroundEstimation(mc_control::MCController & ctl)
{
  if(estimation_loop_.joinable())
  {
    estimation_loop_.join();
  }

  estimation_loop_ = std::thread(
    [this, &ctl]()
    {
      // Return the parent body of the sensor (phalanx)
      const std::string& body_of_sensor = ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name_).parent();
      const sva::PTransformd& X_b_s = ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s();

      //
      {
        const std::lock_guard<std::mutex> lock(estimation_mutex_);
        pc_estimated_ground_points_ = std::make_shared<open3d::geometry::PointCloud>();
        pc_transformed_estimated_ground_points_ = std::make_shared<open3d::geometry::PointCloud>();
        pc_full_ground_reconstructed_points_ = std::make_shared<open3d::geometry::PointCloud>();
      }

      while(!stop_loop_.load())
      {
         // Wait until main() sends data
        std::unique_lock lk(start_estimation_mutex_);
        estimation_condition_.wait(lk, [this]{return new_camera_data_.load();});

        // Get fz from the foot
        const double fz = ctl.robot().bodyForceSensor(body_of_sensor).force().z();
        if(ctl.datastore().has("SoftFootState::GetState"))
        {
          const std::string state = ctl.datastore().call<std::string>("SoftFootState::GetState");
          if(state != desired_state_)
          {
            continue;
          }
        }
        else
        {
          continue;
        }

        std::vector<Eigen::Vector3d> new_camera_points;
        {
          const std::lock_guard<std::mutex> lock(mutex_);
          new_camera_points = points_;
        }

        // Access the position of body name in world coordinates (phalanx position)
        auto start = std::chrono::high_resolution_clock::now();
        const auto& X_0_b = ctl.realRobot().bodyPosW(body_of_sensor);

        {
          ceres::Problem problem;
          double pitch = 0.;
          double t_z = 0.;
          {
            const std::lock_guard<std::mutex> lock(estimation_mutex_);
            ground_points_.clear();
            for(const auto& point: new_camera_points)
            {
              const sva::PTransformd X_0_p = sva::PTransformd(point) * X_b_s * X_0_b;
              ground_points_.push_back(X_0_p.translation());
              const sva::PTransformd X_s_p(point);
              ceres::CostFunction* cost_function = PitchZCostFunctor::Create(X_s_p, X_0_b, X_b_s);
              problem.AddResidualBlock(cost_function,  new ceres::CauchyLoss(0.5), &pitch, &t_z);
            }
          }
          ceres::CostFunction * min_p = new ceres::AutoDiffCostFunction<Minimize, 1, 1>(new Minimize());
          problem.AddResidualBlock(min_p, nullptr, &pitch);
          ceres::CostFunction * min_z = new ceres::AutoDiffCostFunction<Minimize, 1, 1>(new Minimize());
          problem.AddResidualBlock(min_z, nullptr, &t_z);

          ceres::Solver::Options options;
          options.linear_solver_type = ceres::DENSE_QR;
          ceres::Solver::Summary summary;
          ceres::Solve(options, &problem, &summary);

          const sva::PTransformd X_b_b(sva::RotY(pitch).cast<double>(), Eigen::Vector3d(0., 0., t_z));

          // Compute the 3D point in world frame
          {
            const std::lock_guard<std::mutex> lock(estimation_mutex_);
            corrected_ground_points_.clear();
            for(const auto& point: new_camera_points)
            {
              // From camera frame to world frame
              const sva::PTransformd X_s_p(point);
              const sva::PTransformd X_0_p = X_s_p * X_b_s * X_b_b * X_0_b;
              corrected_ground_points_.push_back(X_0_p.translation());
            }
          }
        }

        try
        {
          {
            const std::lock_guard<std::mutex> lock(estimation_mutex_);
            pc_estimated_ground_points_->points_ = corrected_ground_points_;
          }

          if(pc_full_ground_reconstructed_points_->points_.empty())
          {
            const std::lock_guard<std::mutex> lock(estimation_mutex_);
            *pc_full_ground_reconstructed_points_ = *pc_estimated_ground_points_;
            continue;
          }

          {
            const std::lock_guard<std::mutex> lock(estimation_mutex_);
            pc_estimated_ground_points_ = pc_estimated_ground_points_->VoxelDownSample(0.005);
          }

          const auto front = Eigen::Vector3d(X_0_b.translation().x() + 0.0, X_0_b.translation().y() - 0.05, -0.04);
          const auto back = Eigen::Vector3d(Eigen::Vector3d(X_0_b.translation().x() + 0.55, X_0_b.translation().y() + 0.05, 0.04));
          std::shared_ptr<open3d::geometry::PointCloud> pc_ground_points_cropped(new open3d::geometry::PointCloud);
          *pc_ground_points_cropped = *pc_full_ground_reconstructed_points_;
          pc_ground_points_cropped = pc_ground_points_cropped->Crop(open3d::geometry::AxisAlignedBoundingBox(front, back));

          // ICP For matching
          auto result = open3d::pipelines::registration::RegistrationGeneralizedICP(
              *pc_estimated_ground_points_, *pc_ground_points_cropped, 0.05, Eigen::Matrix4d::Identity(),
              open3d::pipelines::registration::TransformationEstimationForGeneralizedICP(),
              open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 15));

          // Apply transformation
          {
            const std::lock_guard<std::mutex> lock(estimation_mutex_);
            *pc_transformed_estimated_ground_points_ = *pc_estimated_ground_points_;
            pc_transformed_estimated_ground_points_->Transform(result.transformation_);
            *pc_full_ground_reconstructed_points_ += *pc_transformed_estimated_ground_points_;
            pc_full_ground_reconstructed_points_ = pc_full_ground_reconstructed_points_->VoxelDownSample(0.005);
            std::sort(pc_full_ground_reconstructed_points_->points_.begin(), pc_full_ground_reconstructed_points_->points_.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return a.x() < b.x(); });
            new_ground_data_ = true;
          }
        }
        catch (const std::runtime_error& e)
        {
          std::cout << e.what() << std::endl;
        }
        auto stop = std::chrono::high_resolution_clock::now();
        ground_reconstruction_duration_ = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();
      }
    }
  );
}

} // namespace lipm_walking
EXPORT_OBSERVER_MODULE("CameraSensor", lipm_walking::CameraSensor)