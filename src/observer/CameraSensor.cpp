#include <lipm_walking/observer/CameraSensor.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/version.h>
#include <mc_control/MCController.h>
#include <mc_mujoco/devices/RangeSensor.h>
#include <chrono>
#include <fstream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>

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

void CameraSensor::reset(const mc_control::MCController &)
{
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
  if(new_data_.load())
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).update(points_, 0.);
    new_data_ = false;
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
  logger.addLogEntry(category + "_points_x", [this]() -> std::vector<double>
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::vector<double> d;
    std::transform(points_.begin(), points_.end(), std::back_inserter(d),
      [](const Eigen::Vector3d & v) { return v.x(); });
    return d;
  });
  logger.addLogEntry(category + "_points_y", [this]() -> std::vector<double>
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::vector<double> d;
    std::transform(points_.begin(), points_.end(), std::back_inserter(d),
      [](const Eigen::Vector3d & v) { return v.y(); });
    return d;
  });
  logger.addLogEntry(category + "_points_z", [this]() -> std::vector<double>
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::vector<double> d;
    std::transform(points_.begin(), points_.end(), std::back_inserter(d),
      [](const Eigen::Vector3d & v) { return v.z(); });
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
    mc_rtc::gui::Trajectory("Points",
      [this, &ctl]()
      {
        std::vector<Eigen::Vector3d> points;
        {
          const std::lock_guard<std::mutex> lock(mutex_);
          points = points_;
        }

        if(points.empty())
        {
          return std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
        }

        const std::string& body_of_sensor = ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name_).parent();
        // Access the position of body name in world coordinates (phalanx position)
        sva::PTransformd X_0_ph = ctl.realRobot().bodyPosW(body_of_sensor);
        // Returns the transformation from the parent body to the sensor
        const sva::PTransformd& X_ph_s = ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s();
        // Keep the estimated 3d point for the ground
        std::vector<Eigen::Vector3d> traj;
        for(const auto& point: points)
        {
          const sva::PTransformd X_0_m = sva::PTransformd(point) * X_ph_s * X_0_ph;
          traj.push_back(X_0_m.translation());
        }
        return traj;
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
            if(std::abs(ddepth - pixel_depth) > outlier_threshold_)
            {
              return 0.f;
            }
          }
        }
        return sum_depth / counter;
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

        // Get depth frame
        auto frame = frames.get_depth_frame();
        frame = frame.apply_filter(dec);
        frame = frame.apply_filter(hdr);
        frame = frame.apply_filter(thresh);
        frame = frame.apply_filter(depth2disparity);
        frame = frame.apply_filter(spat);
        frame = frame.apply_filter(disparity2depth);

        rs2_intrinsics intrinsics = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

        const size_t height = frame.get_height();
        const size_t width = frame.get_width();

        const size_t half_height = static_cast<size_t>(static_cast<double>(height) * 0.5);
        const size_t half_width = static_cast<size_t>(static_cast<double>(width) * 0.5);


        std::array<float, 2> pixel;
        const int half_kernel_size = static_cast<int>(kernel_size_/2);
        std::vector<Eigen::Vector3d> points;
        for(size_t i = half_width; i < width - half_kernel_size; ++i)
        {
          pixel[0] = static_cast<float>(i);
          pixel[1] = static_cast<float>(half_height);

          const float depth = applyKernel(pixel, frame);
          const Eigen::Vector3d point = depthToPoint(intrinsics, pixel, depth);
          if(point.z() != 0)
          {
            if(points.empty())
            {
              points.push_back(point);
              // mc_rtc::log::error("{} {} {}", point.x(), point.y(), point.z());
            }
            // Add a post check for noisy data
            else if((points[i] - points[i-1]).norm() < 0.03)
            {
              points.push_back(point);
              // mc_rtc::log::info("{} {} {}", point.x(), point.y(), point.z());
            }
          }
        }

        {
          const std::lock_guard<std::mutex> lock(mutex_);
          points_ = points;
        }

        // Needs to update
        new_data_ = true;
      }
    }
  );
}

} // namespace lipm_walking
EXPORT_OBSERVER_MODULE("CameraSensor", lipm_walking::CameraSensor)