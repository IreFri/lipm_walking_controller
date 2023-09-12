#include "CameraSensorServer.h"

#include "CameraSensorCeresFunctions.h"

// Include RealSense Cross Platform API
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <fstream>
#include <thread>

#include <signal.h>
#include <unistd.h>

namespace lipm_walking
{

CameraSensorServer::CameraSensorServer(const char * name, const mc_rtc::Configuration & config)
{
  name_ = name;
  data_ = CameraSensorShared::get(name);
  if(data_->server.isAlive())
  {
    mc_rtc::log::error_and_throw("[CameraSensorServer] Another server (pid: {}) is already handling {}",
                                 data_->server.pid, name);
  }
  data_->newServer(getpid());

  auto serial = config("camera_serial");
  if(serial.isString())
  {
    camera_serial_ = serial.operator std::string();
  }
  else
  {
    camera_serial_ = std::to_string(serial.operator uint64_t());
  }

  if(config.has("path_to_preset"))
  {
    path_to_preset_ = config("path_to_preset").operator std::string();
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
  mc_rtc::log::info("[CameraSensorServer::{}] 'camera_serial' is {}", name_, camera_serial_);
  mc_rtc::log::info("[CameraSensorServer::{}] 'path_to_preset' is {}", name_, path_to_preset_);
  mc_rtc::log::info("[CameraSensorServer::{}] 'kernel_size' is {}", name_, kernel_size_);
  mc_rtc::log::info("[CameraSensorServer::{}] 'kernel_threshold' is {}", name_, kernel_threshold_);
  mc_rtc::log::info("[CameraSensorServer::{}] 'outlier_threshold' is {}", name_, outlier_threshold_);
}

CameraSensorServer::~CameraSensorServer()
{
  stop();
}

void CameraSensorServer::run()
{
  if(acquisition_th_.joinable())
  {
    mc_rtc::log::error_and_throw("[CameraSensorServer::{}] Already running, call stop() first", name_);
  }
  run_ = true;
  acquisition_th_ = std::thread([this]() { acquisition(); });
  computation_th_ = std::thread([this]() { computation(); });
}

void CameraSensorServer::stop()
{
  run_ = false;
  if(computation_th_.joinable())
  {
    computation_th_.join();
  }
  if(acquisition_th_.joinable())
  {
    if(pipeline_started_)
    {
      acquisition_th_.join();
    }
    else
    {
      acquisition_th_.detach();
      mc_rtc::log::critical("The acquisition pipeline has not started yet, this might not exit gracefully");
    }
  }
}

void CameraSensorServer::acquisition()
{
  rs2::config cfg;
  // Check if we read from a .bag or from a camera
  if(!path_to_replay_data_.empty())
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
  pipeline_started_ = true;

  // Load preset
  if(!path_to_preset_.empty())
  {
    std::ifstream t(path_to_preset_);
    std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    rs2::device dev = prof.get_device();
    auto advanced = dev.as<rs400::advanced_mode>();
    advanced.load_json(str);
  }

  auto depthToPoint = [](const rs2_intrinsics & intrinsics, const std::array<float, 2> & pixel,
                         float depth) -> Eigen::Vector3d
  {
    Eigen::Vector3f point;
    rs2_deproject_pixel_to_point(point.data(), &intrinsics, pixel.data(), depth);
    return point.cast<double>();
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

  while(run_)
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

    const size_t height = static_cast<size_t>(frame.get_height());
    const size_t width = static_cast<size_t>(frame.get_width());

    mc_rtc::log::info("h {} w {}", height, width);

    const size_t half_height = static_cast<size_t>(static_cast<double>(height) * 0.5);
    const size_t half_width = static_cast<size_t>(static_cast<double>(width) * 0.5);

    std::array<float, 2> pixel;
    const size_t half_kernel_size = kernel_size_ / 2;
    std::vector<Eigen::Vector3d> points;
    for(size_t i = half_kernel_size; i < width - half_kernel_size; ++i)
    {
      pixel[0] = static_cast<float>(i);
      for(int j = -1; j <= 1; ++j)
      {
        pixel[1] = static_cast<float>(half_height + j);

        const float depth = frame.get_distance(static_cast<int>(pixel[0]), static_cast<int>(pixel[1]));
        Eigen::Vector3d point = depthToPoint(intrinsics, pixel, depth);
        if(point.z() != 0)
        {
          // point.y() = 0.;
          points.push_back(point);
        }
      }
    }

    {
      const std::lock_guard<std::mutex> lock(points_mtx_);
      points_ = points;
    }

    {
      ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
      std::cout << "Updating data_->points" << std::endl;
      data_->points.resize(points.size());
      for(size_t i = 0; i < points.size(); ++i)
      {
        data_->points[i] = points[i];
      }
      std::cout << "New size is " << data_->points.size() << std::endl;
    }

    if(data_->client.isAlive())
    {
      data_->data_ready->notify_all();
    }
  }
}

void CameraSensorServer::computation()
{
  bool client_was_alive = data_->client.isAlive();
  while(run_)
  {
    while(data_->client.isAlive())
    {
      if(!client_was_alive)
      {
        mc_rtc::log::success("[CameraSensorServer::{}] Client connected", name_);
        client_was_alive = true;
      }
      ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->mtx);
      if(data_->compute_ready->timed_wait(lck, pclock::universal_time() + ptime::seconds(1)))
      {
        // mc_rtc::log::error("Here");
        do_computation();
      }
    }
    if(client_was_alive)
    {
      client_was_alive = false;
      mc_rtc::log::critical("[CameraSensorServer::{}] Client disconnected", name_);
    }
    // Wait for the client to become available
    data_->server_idle.notify_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void CameraSensorServer::do_computation()
{
  if(data_->skip)
  {
    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// fromCameraToWorldFrameWithAlignement
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  auto fromCameraToWorldFrameWithAlignement = [this](const std::vector<Eigen::Vector3d> & camera_points, double pitch, double t_z)
  {
    std::vector<Eigen::Vector3d> ground_points;
    const sva::PTransformd X_b_b(sva::RotY(pitch).cast<double>(), Eigen::Vector3d(0., 0., t_z));

    for(int i = camera_points.size() - 1; i >= 0; --i)
    {
      const sva::PTransformd X_s_p(camera_points[i]);
      const sva::PTransformd X_0_p = X_s_p * data_->X_b_s * X_b_b * data_->X_0_b;
      if(X_0_p.translation().x() - data_->X_0_b.translation().x() < 0.55)
      {
        ground_points.push_back(X_0_p.translation());
      }
    }

    return ground_points;
  };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// selectLineForCeresAlignement
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  auto selectLineForCeresAlignement = [this](const std::vector<Eigen::Vector3d> & ground_points)
  {
    std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud);
    pc->points_ = ground_points;
    std::sort(pc->points_.begin(), pc->points_.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });
    pc->EstimateNormals();

    std::vector<Eigen::Vector3d> selected_line;
    if(pc->normals_.empty())
    {
      return selected_line;
    }

    pc->OrientNormalsTowardsCameraLocation(data_->X_0_b.translation());

    std::vector<std::vector<Eigen::Vector3d>> lines;
    lines.push_back(std::vector<Eigen::Vector3d>{});

    const auto n_z = Eigen::Vector3d::UnitZ();

    for(size_t i = 0; i < pc->points_.size(); ++i)
    {
      const auto & n_0 = pc->normals_[i];

      const double angle = std::acos(n_0.dot(n_z) / (n_0.norm() * n_z.norm()));
      if((std::abs(angle * 180. / M_PI) > 15.0)
        || ((!lines.back().empty() && (lines.back().back().x() - pc->points_[i].x()) > 0.02))
        || ((!lines.back().empty() && std::abs(lines.back().back().z() - pc->points_[i].z()) > 0.005)))
      {
        lines.push_back(std::vector<Eigen::Vector3d>{});
      }
      lines.back().push_back(pc->points_[i]);
    }

    for(const auto & line: lines)
    {
      if(line.size() > 30)
      {
        selected_line.insert(selected_line.end(), line.begin(), line.end());
      }
    }

    return selected_line;
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// computePitchAndTzWithCeres
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  auto computePitchAndTzWithCeres = [this](const std::vector<Eigen::Vector3d> & selected_line, double & pitch, double & t_z)
  {
    pitch = 0.;
    t_z = 0.;

    ceres::Problem problem;
    for(const auto& T_0_p: selected_line)
    {
      const sva::PTransformd X_0_p(T_0_p);
      const sva::PTransformd X_s_p = X_0_p * (data_->X_b_s * data_->X_0_b).inv();
      ceres::CostFunction* cost_function = PitchZCostFunctor::Create(X_s_p, data_->X_0_b, data_->X_b_s);
      problem.AddResidualBlock(cost_function,  new ceres::CauchyLoss(0.5), &pitch, &t_z);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// reAlignGround
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  auto reAlignGround = [this](const std::vector<Eigen::Vector3d> & ground_points, double pitch, double t_z)
  {
    std::vector<Eigen::Vector3d> _ground_points;
    const sva::PTransformd X_b_b(sva::RotY(pitch).cast<double>(), Eigen::Vector3d(0., 0., t_z));
    for(const auto& T_0_p: ground_points)
    {
      // From camera frame to world frame
      const sva::PTransformd _X_0_p(T_0_p);
      const sva::PTransformd X_s_p = _X_0_p * (data_->X_b_s * data_->X_0_b).inv();
      const sva::PTransformd X_0_p = X_s_p * data_->X_b_s * X_b_b * data_->X_0_b;
      if(X_0_p.translation().z() > -0.005)
      {
        _ground_points.push_back(X_0_p.translation());
      }
    }

    std::sort(_ground_points.begin(), _ground_points.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });

    return _ground_points;
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// filtering
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  auto filtering = [](const std::vector<Eigen::Vector3d> & _ground_points)
  {
    auto ground_points = _ground_points;

    std::sort(ground_points.begin(), ground_points.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });

    std::vector<std::vector<Eigen::Vector3d>> groups;
    groups.push_back(std::vector<Eigen::Vector3d>{});

    double threshold = 0.002;
    for(size_t i = 1; i < ground_points.size(); ++i)
    {
      const Eigen::Vector3d & T_0_p_0 = ground_points[i - 1];
      const Eigen::Vector3d & T_0_p_1 = ground_points[i];

      const double d_x = std::abs(T_0_p_1.x() - T_0_p_0.x());
      const double d_z = std::abs(T_0_p_1.z() - T_0_p_0.z());

      if(d_x > threshold || d_z > threshold)
      {
        if(groups.back().size() > 10)
        {
          threshold += 0.001;
        }

        if(groups.back().size() < 50)
        {
          groups.back().clear();
        }
        else
        {
          groups.push_back(std::vector<Eigen::Vector3d>{});
        }
      }
      groups.back().push_back(T_0_p_1);
    }

    std::vector<Eigen::Vector3d> new_ground_points;
    for(auto & group: groups)
    {
      if(group.size() >= 50)
      {
        group.resize(group.size() - 10);
        new_ground_points.insert(new_ground_points.end(), group.begin(), group.end());
      }
    }

    return new_ground_points;
  };


  {
    ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
    if(data_->reset_ground)
    {
      ground_points_.clear();
      pre_new_ground_points_.clear();
      data_->ground_points.clear();
      data_->reset_ground = false;
    }
  }

  {
    mc_rtc::log::info("Acquired {} raw points", points_.size());
    const std::lock_guard<std::mutex> lock(points_mtx_);
    new_camera_points_ = points_;
    new_ground_points_.reserve(new_camera_points_.size());
  }

  if(new_camera_points_.empty())
  {
    return;
  }

  auto start = std::chrono::high_resolution_clock::now();
  {
    auto start = std::chrono::high_resolution_clock::now();

    pre_new_ground_points_ = fromCameraToWorldFrameWithAlignement(new_camera_points_, previous_pitch_, previous_t_z_);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [Align using previous pitch and t_z]", static_cast<double>(duration.count()) / 1000.);
  }

  std::vector<Eigen::Vector3d> selected_line;
  {
    auto start = std::chrono::high_resolution_clock::now();

    selected_line = selectLineForCeresAlignement(pre_new_ground_points_);

    {
      ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
      data_->selected_points.resize(selected_line.size());
      for(size_t i = 0; i < selected_line.size(); ++i)
      {
        data_->selected_points[i] = selected_line[i];
      }
      data_->result_ready->notify_all();
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [Select line for Ceres]", static_cast<double>(duration.count()) / 1000.);
  }

    // If no selected points, we skip the estimation
  if(selected_line.empty())
  {
    mc_rtc::log::info("Skip the estimation as there are no selected line for the alignement");
    data_->result_ready->notify_all();
    return;
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    computePitchAndTzWithCeres(selected_line, previous_pitch_, previous_t_z_);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [Compute pitch and t_z]", static_cast<double>(duration.count()) / 1000.);
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    new_ground_points_ = reAlignGround(pre_new_ground_points_, previous_pitch_, previous_t_z_);

    {
      ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
      data_->aligned_points.resize(new_ground_points_.size());
      for(size_t i = 0; i < new_ground_points_.size(); ++i)
      {
        data_->aligned_points[i] = new_ground_points_[i];
      }
      data_->result_ready->notify_all();
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [Realignement of ground]", static_cast<double>(duration.count()) / 1000.);
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    new_ground_points_ = filtering(new_ground_points_);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [Filtering]", static_cast<double>(duration.count()) / 1000.);
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    std::shared_ptr<open3d::geometry::PointCloud> target(new open3d::geometry::PointCloud);
    target->points_ = ground_points_;

    const auto front = Eigen::Vector3d(data_->X_0_b.translation().x() + 0.0, data_->X_0_b.translation().y() - 0.10, -0.10);
    const auto back = Eigen::Vector3d(Eigen::Vector3d(data_->X_0_b.translation().x() + 0.55, data_->X_0_b.translation().x() + 0.10, 0.10));
    target = target->Crop(open3d::geometry::AxisAlignedBoundingBox(front, back));

    if(!new_ground_points_.empty() && target->points_.size() > 10)
    {
      std::shared_ptr<open3d::geometry::PointCloud> source(new open3d::geometry::PointCloud);
      source->points_ = new_ground_points_;

      const double bary_source = std::accumulate(source->points_.begin(), source->points_.end(), 0., [](const double s, const Eigen::Vector3d & v) { return s + v.y(); }) / source->points_.size();
      const double bary_target = std::accumulate(target->points_.begin(), target->points_.end(), 0., [](const double s, const Eigen::Vector3d & v) { return s + v.y(); }) / target->points_.size();

      // Re-align
      for(auto & point: source->points_)
      {
        point.y() += (bary_target - bary_source);
      }

      // ICP For matching
      auto result = open3d::pipelines::registration::RegistrationGeneralizedICP(
          *source, *target, 0.05, Eigen::Matrix4d::Identity(),
          open3d::pipelines::registration::TransformationEstimationForGeneralizedICP(),
          open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 15));

      // Apply transformation
      source->Transform(result.transformation_);

      bool success = true;
      if(std::abs(mc_rbdyn::rpyFromMat(result.transformation_.block<3, 3>(0, 0)).y()) > 0.0872665 ||
        std::abs(mc_rbdyn::rpyFromMat(result.transformation_.block<3, 3>(0, 0)).z()) > 0.0872665)
      {
        mc_rtc::log::error("Discard ICP result as the angle is > 5 [deg]");
        success = false;
      }
      else
      {
        auto distances = source->ComputePointCloudDistance(*target);
        std::sort(distances.begin(), distances.end());
        double sum = 0.;
        for(size_t i = 0; i < distances.size(); ++i)
        {
          sum += std::abs(distances[i]);
        }
        sum /= distances.size();

        if(sum < 0.005)
        {
          new_ground_points_ = source->points_;
        }
        else
        {
          mc_rtc::log::error("Do not use ICP result because {} > 0.005", sum);
          success = false;
        }
      }

      if(!success)
      {
        // Re-align
        for(auto & point: new_ground_points_)
        {
          point.y()  += (bary_target - bary_source);
        }
        std::sort(new_ground_points_.begin(), new_ground_points_.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });

        size_t idx_start = 0;
        while(idx_start < ground_points_.size() && new_ground_points_.front().x() > ground_points_[idx_start].x())
        {
          ++ idx_start;
        }

        size_t idx_end = idx_start;
        while(idx_end < ground_points_.size() && new_ground_points_.back().x() > ground_points_[idx_end].x())
        {
          ++ idx_end;
        }

        ground_points_.erase(ground_points_.begin() + idx_start, ground_points_.begin() + idx_end);
      }
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [ICP Alignement]", static_cast<double>(duration.count()) / 1000.);
  }

  selected_line = selectLineForCeresAlignement(new_ground_points_);
  {
    ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
    data_->selected_points.resize(selected_line.size());
    for(size_t i = 0; i < selected_line.size(); ++i)
    {
      data_->selected_points[i] = selected_line[i];
    }
    data_->result_ready->notify_all();
  }

  // Check if obstacles
  bool obstacles = false;
  for(const auto & point: new_ground_points_)
  {
    if(point.z() > 0.01)
    {
      obstacles = true;
      break;
    }
  }

  if(!obstacles)
  {
    // Re-Alignement to avoid drift after ICP
    // Re-align ICP Results
    auto start = std::chrono::high_resolution_clock::now();

    // If no selected points, we skip the estimation
    if(selected_line.empty())
    {
      mc_rtc::log::info("Skip the estimation as there are no selected line for the alignement");
      data_->result_ready->notify_all();
      return;
    }

    double pitch = 0.;
    double t_z = 0.;

    computePitchAndTzWithCeres(selected_line, pitch, t_z);

    new_ground_points_ = reAlignGround(new_ground_points_, pitch, t_z);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [Realignement of ground after ICP]", static_cast<double>(duration.count()) / 1000.);
  }

  ground_points_.insert(ground_points_.end(), new_ground_points_.begin(), new_ground_points_.end());

  {
    std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud);
    pc->points_ = ground_points_;
    pc = pc->VoxelDownSample(0.002);
    ground_points_ = pc->points_;
  }

  std::sort(ground_points_.begin(), ground_points_.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });

  std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud);
  pc->points_ = ground_points_;
  pc = pc->VoxelDownSample(0.005);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  mc_rtc::log::info("Took {} ms for [E V E R Y T H I N G]", static_cast<double>(duration.count()) / 1000.);

  {
    ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
    const auto & points = pc->points_;
    data_->ground_points.resize(points.size());
    for(size_t i = 0; i < points.size(); ++i)
    {
      data_->ground_points[i] = points[i];
    }
  }
  data_->result_ready->notify_all();
}

} // namespace lipm_walking

std::mutex cv_mtx;
std::condition_variable cv;
std::atomic<bool> stop_program{false};

void signal_callback_handler(int)
{
  stop_program = true;
  cv.notify_all();
}

int main(int argc, char * argv[])
{
  if(argc < 3)
  {
    mc_rtc::log::info("[usage] {} <ControllerConfig.yaml> <CameraName>", argv[0]);
    return 1;
  }
  mc_rtc::Configuration ctl_cfg(argv[1]);
  std::string camera = argv[2];
  if(!ctl_cfg.has("ObserverPipelines"))
  {
    mc_rtc::log::error_and_throw("Must have ObserverPipelines in the configuration");
  }
  auto observer_pipelines = ctl_cfg("ObserverPipelines");
  auto observers =
      [&]()
  {
    if(observer_pipelines.isArray())
    {
      mc_rtc::log::warning("Multiple pipelines in configuration, taking the first one");
      return observer_pipelines[0]("observers");
    }
    return observer_pipelines("observers");
  }()
          .
      operator std::vector<mc_rtc::Configuration>();
  auto camera_cfg = [&]()
  {
    for(const auto & o : observers)
    {
      if(o.has("name") && o("name").operator std::string() == camera)
      {
        return o("config");
      }
    }
    mc_rtc::log::error_and_throw("No configuration for camera {} found in {}", camera, argv[1]);
  }();
  lipm_walking::CameraSensorServer server(camera.c_str(), camera_cfg);
  mc_rtc::log::info("[CameraServer::{}] Running, press ctrl+c to stop", camera);
  server.run();
  signal(SIGINT, signal_callback_handler);
  std::unique_lock<std::mutex> lck(cv_mtx);
  cv.wait(lck, []() { return stop_program.load(); });
  server.stop();
  return 0;
}
