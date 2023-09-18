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
  auto fromCameraToWorldFrameWithAlignement = [this](const std::vector<Eigen::Vector3d> & camera_points)
  {
    std::vector<Eigen::Vector3d> ground_points;
    for(int i = camera_points.size() - 1; i >= 0; --i)
    {
      const sva::PTransformd X_s_p(camera_points[i]);
      sva::PTransformd X_0_p = X_s_p * data_->X_b_s * X_0_b_;
      X_0_p.translation().y() = 0.;
      ground_points.push_back(X_0_p.translation());
    }

    std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud);
    pc->points_ = ground_points;
    pc = pc->VoxelDownSample(0.0025);
    ground_points = pc->points_;
    std::sort(ground_points.begin(), ground_points.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });

    return ground_points;
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// thresholdBasedOnDistance
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  auto thresholdBasedOnDistance = [&](double start_x, double X_0_p_x)
  {
    const double min_x = start_x;
    const double max_x = start_x + 0.65;
    const double min_threshold = 0.002;
    const double max_threshold = 0.030;

    return min_threshold + (X_0_p_x  - min_x) * (max_threshold - min_threshold) / (max_x - min_x);
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// splitGroundAndObstacles
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  auto splitGroundAndObstacles = [this](const std::vector<Eigen::Vector3d> & ground_points, double threshold_deg,
    std::vector<Eigen::Vector3d> & grounds, std::vector<Eigen::Vector3d> & obstacles) -> void
  {
    std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud);
    pc->points_ = ground_points;
    for(const auto & p: ground_points)
    {
      pc->points_.push_back(Eigen::Vector3d(p.x(), 0.002, p.z()));
      pc->points_.push_back(Eigen::Vector3d(p.x(), 0.002, p.z()));
    }

    pc->EstimateNormals();

    if(pc->normals_.empty())
    {
      return;
    }

    try
    {
      pc->OrientNormalsTowardsCameraLocation(X_0_b_.translation());
    }
    catch(const std::exception &)
    {
      return;
    }

    bool is_obstacle = false;
    const auto n_z = Eigen::Vector3d::UnitZ();
    for(size_t i = 1; i < ground_points.size(); ++i)
    {
      const auto & n_0 = pc->normals_[i];
      const double angle = std::acos(n_0.dot(n_z) / (n_0.norm() * n_z.norm()));

      if(is_obstacle && std::abs(pc->points_[i].x() - pc->points_[i - 1].x()) > 0.01)
      {
        is_obstacle = false;
      }

      if(is_obstacle || std::abs(angle * 180. / M_PI) > threshold_deg)
      {
        is_obstacle = true;
        obstacles.push_back(pc->points_[i]);
      }
      else
      {
        grounds.push_back(pc->points_[i]);
      }
    }
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// selectObstacles
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  auto selectObstacles = [&](std::vector<Eigen::Vector3d> & ground_points, bool with_erase = false)
  {
    std::vector<size_t> idxs_to_erase;

    std::vector<Eigen::Vector3d> res;
    for(int i = 0; i < ground_points.size() - 1; ++ i)
    {
      const auto & p = ground_points[i];
      if(p.z() > 0.002)
      {
        res.push_back(p);
        if(std::abs(ground_points[i + 1].x() - p.x()) > 0.02)
        {
          idxs_to_erase.push_back(i);
          idxs_to_erase.push_back(i - 1);
          idxs_to_erase.push_back(i - 2);
          idxs_to_erase.push_back(i - 3);
        }
      }
    }

    if(with_erase)
    {
      for(const auto & index: idxs_to_erase)
      {
        ground_points.erase(ground_points.begin() + index);
      }
    }

    return res;
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
      const sva::PTransformd X_s_p = X_0_p * (data_->X_b_s * X_0_b_).inv();
      ceres::CostFunction* cost_function = PitchZCostFunctor::Create(X_s_p, X_0_b_, data_->X_b_s);
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
  auto reAlignGround = [this](const std::vector<Eigen::Vector3d> & ground_points, double pitch, double t_z, double cutoff_z)
  {
    std::vector<Eigen::Vector3d> _ground_points;
    const sva::PTransformd X_b_b(sva::RotY(pitch).cast<double>(), Eigen::Vector3d(0., 0., t_z));
    for(const auto& T_0_p: ground_points)
    {
      // From camera frame to world frame
      const sva::PTransformd _X_0_p(T_0_p);
      const sva::PTransformd X_s_p = _X_0_p * (data_->X_b_s * X_0_b_).inv();
      const sva::PTransformd X_0_p = X_s_p * data_->X_b_s * X_b_b * X_0_b_;
      if(X_0_p.translation().z() > cutoff_z)
      {
        _ground_points.push_back(X_0_p.translation());
      }
    }

    return _ground_points;
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// denoise
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  auto denoise = [this, &thresholdBasedOnDistance](std::vector<Eigen::Vector3d> points)
  {
    std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud);
    pc->points_ = points;

    open3d::geometry::KDTreeFlann tree(*pc);

    // double threshold = 0.0055;
    // double add_threshold = 0.0005;
    const double add_threshold_searching = 0.0005;

    std::vector<int> indices_done;

    std::vector<std::vector<Eigen::Vector3d>> segmentation;
    segmentation.push_back(std::vector<Eigen::Vector3d>{});

    for(size_t i = 0; i < points.size(); ++i)
    {
      if(std::find(indices_done.begin(), indices_done.end(), i) != indices_done.end())
      {
        continue;
      }

      std::vector<int> current_indices;

      Eigen::Vector3d point = points[i];
      size_t nr_failed = 0;
      size_t max_failure = 3;
      while(true)
      {
        double current_threshold = thresholdBasedOnDistance(points.front().x(), point.x());

        std::vector<int> indices;
        std::vector<double> distances;
        tree.SearchRadius<Eigen::Vector3d>(point, current_threshold, indices, distances);

        if(indices.size() == 1 && nr_failed < max_failure)
        {
          current_threshold += add_threshold_searching;
          ++ nr_failed;
        }
        else if(indices.size() == 1)
        {
          break;
        }

        std::sort(indices.begin(), indices.end());
        bool are_new = false;
        for(const auto & idx: indices)
        {
          const bool is_new = std::find(indices_done.begin(), indices_done.end(), idx) == indices_done.end();
          if(is_new)
          {
            current_indices.push_back(idx);
            point = points[idx];
          }
          are_new = are_new || is_new;
        }

        if(!are_new)
        {
          break;
        }

        indices_done.insert(indices_done.end(), current_indices.begin(), current_indices.end());
      }

      for(const auto & idx: current_indices)
      {
        segmentation.back().push_back(points[idx]);
      }

      double threshold = thresholdBasedOnDistance(points.front().x(), point.x());
      segmentation.push_back(std::vector<Eigen::Vector3d>{});

    }

    std::vector<Eigen::Vector3d> res;
    for(const auto & group: segmentation)
    {
      if(group.size() >= 10)
      {
        res.insert(res.end(), group.begin(), group.end());
      }
    }

    return res;
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

  auto start_computation = std::chrono::high_resolution_clock::now();
  {
    auto start = std::chrono::high_resolution_clock::now();

    X_0_b_ = data_->X_0_b;
    X_0_b_.translation().y() = 0.;

    const Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(data_->X_0_b.rotation());
    X_0_b_.rotation() = mc_rbdyn::rpyToMat(Eigen::Vector3d(rpy(0), 0., 0.));

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [Extract X_0_b]", static_cast<double>(duration.count()) / 1000.);
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    pre_new_ground_points_ = fromCameraToWorldFrameWithAlignement(new_camera_points_);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [From Camera to World frame]", static_cast<double>(duration.count()) / 1000.);
  }
  {
    auto stop_computation = std::chrono::high_resolution_clock::now();
    auto duration_computation = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
    if(duration_computation.count() / 1000. > 33.0)
    {
      mc_rtc::log::error("Timeout");
      data_->result_ready->notify_all();
      return;
    }
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    pre_new_ground_points_ = denoise(pre_new_ground_points_);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [Denoise]", static_cast<double>(duration.count()) / 1000.);
  }
  {
    auto stop_computation = std::chrono::high_resolution_clock::now();
    auto duration_computation = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
    if(duration_computation.count() / 1000. > 33.0)
    {
      mc_rtc::log::error("Timeout");
      data_->result_ready->notify_all();
      return;
    }
  }

   // If not enough points
  if(pre_new_ground_points_.size() < 50)
  {
    mc_rtc::log::info("Skip the estimation as there are not enough points after denoising");
    data_->result_ready->notify_all();
    return;
  }

  std::vector<Eigen::Vector3d> grounds;
  std::vector<Eigen::Vector3d> obstacles;
  {
    auto start = std::chrono::high_resolution_clock::now();

    splitGroundAndObstacles(pre_new_ground_points_, 15.0, grounds, obstacles);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [Select line for Ceres]", static_cast<double>(duration.count()) / 1000.);
  }
  {
    auto stop_computation = std::chrono::high_resolution_clock::now();
    auto duration_computation = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
    if(duration_computation.count() / 1000. > 33.0)
    {
      mc_rtc::log::error("Timeout");
      data_->result_ready->notify_all();
      return;
    }
  }

  // If no selected points, we skip the estimation
  if(grounds.empty())
  {
    mc_rtc::log::info("Skip the estimation as there are no selected grounds for the alignement");
    data_->result_ready->notify_all();
    return;
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    computePitchAndTzWithCeres(grounds, previous_pitch_, previous_t_z_);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [Compute pitch and t_z]", static_cast<double>(duration.count()) / 1000.);
  }
  {
    auto stop_computation = std::chrono::high_resolution_clock::now();
    auto duration_computation = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
    if(duration_computation.count() / 1000. > 33.0)
    {
      mc_rtc::log::error("Timeout");
      data_->result_ready->notify_all();
      return;
    }
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    new_ground_points_ = reAlignGround(pre_new_ground_points_, previous_pitch_, previous_t_z_, -0.001);

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
    auto stop_computation = std::chrono::high_resolution_clock::now();
    auto duration_computation = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
    if(duration_computation.count() / 1000. > 33.0)
    {
      mc_rtc::log::error("Timeout");
      data_->result_ready->notify_all();
      return;
    }
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    grounds.clear();
    obstacles.clear();
    splitGroundAndObstacles(pre_new_ground_points_, 5.0, grounds, obstacles);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [Select line for Ceres]", static_cast<double>(duration.count()) / 1000.);
  }
  {
    auto stop_computation = std::chrono::high_resolution_clock::now();
    auto duration_computation = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
    if(duration_computation.count() / 1000. > 33.0)
    {
      mc_rtc::log::error("Timeout");
      data_->result_ready->notify_all();
      return;
    }
  }

  // If no selected points, we skip the estimation
  if(grounds.empty())
  {
    mc_rtc::log::info("Skip the estimation as there are no selected grounds for the alignement");
    data_->result_ready->notify_all();
    return;
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    computePitchAndTzWithCeres(grounds, previous_pitch_, previous_t_z_);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [Compute pitch and t_z]", static_cast<double>(duration.count()) / 1000.);
  }
  {
    auto stop_computation = std::chrono::high_resolution_clock::now();
    auto duration_computation = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
    if(duration_computation.count() / 1000. > 33.0)
    {
      mc_rtc::log::error("Timeout");
      data_->result_ready->notify_all();
      return;
    }
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    new_ground_points_ = reAlignGround(pre_new_ground_points_, previous_pitch_, previous_t_z_, -0.001);

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
    auto stop_computation = std::chrono::high_resolution_clock::now();
    auto duration_computation = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
    if(duration_computation.count() / 1000. > 33.0)
    {
      mc_rtc::log::error("Timeout");
      data_->result_ready->notify_all();
      return;
    }
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    std::shared_ptr<open3d::geometry::PointCloud> target(new open3d::geometry::PointCloud);
    target->points_ = ground_points_;

    const auto front = Eigen::Vector3d(X_0_b_.translation().x() + 0.0, X_0_b_.translation().y() - 0.10, -0.10);
    const auto back = Eigen::Vector3d(Eigen::Vector3d(X_0_b_.translation().x() + 0.55, X_0_b_.translation().x() + 0.10, 0.10));
    target = target->Crop(open3d::geometry::AxisAlignedBoundingBox(front, back));

    // Only keep 0.15m of data
    size_t idx_start = 0;
    while(idx_start < new_ground_points_.size() && (new_ground_points_[idx_start].x() - new_ground_points_.front().x()) < 0.15)
    {
      ++ idx_start;
    }
    new_ground_points_.erase(new_ground_points_.begin() + idx_start, new_ground_points_.end());

    // If not enough points
    if(new_ground_points_.size() < 20)
    {
      mc_rtc::log::info("Skip the estimation as there are not enough points after shrinking down to 0.15m");
      new_ground_points_.clear();
    }

    if(!new_ground_points_.empty() && !obstacles.empty() && target->points_.size() > 10)
    {
      std::sort(new_ground_points_.begin(), new_ground_points_.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });

      std::vector<Eigen::Vector3d> target_obstacles = selectObstacles(target->points_);
      std::vector<Eigen::Vector3d> src_obstacles = selectObstacles(new_ground_points_, true);

      double t_x = 0.;
      ceres::Problem problem;

      if(target_obstacles.size() > src_obstacles.size())
      {
        for(const auto& T_0_p: src_obstacles)
        {
          ceres::CostFunction* cost_function = XCostFunctor::Create(T_0_p, target_obstacles);
          problem.AddResidualBlock(cost_function,  new ceres::CauchyLoss(0.5), &t_x);
        }
      }
      else
      {
        for(const auto& T_0_p: target_obstacles)
        {
          ceres::CostFunction* cost_function = XCostFunctor::Create(T_0_p, src_obstacles);
          problem.AddResidualBlock(cost_function,  new ceres::CauchyLoss(0.5), &t_x);
        }
      }

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);

      if(!(target_obstacles.size() > src_obstacles.size()))
      {
        t_x = -t_x;
      }

      if(std::abs(t_x) > 0.025)
      {
        mc_rtc::log::error("Too much error in x: {}", t_x);
        new_ground_points_.clear();
      }

      if(!new_ground_points_.empty())
      {
        for(auto & p: new_ground_points_)
        {
          p.x() += t_x;
        }
      }
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [ICP Alignement]", static_cast<double>(duration.count()) / 1000.);
  }

  if(historic_points_.empty())
  {
    historic_points_ = new_ground_points_;
  }

  if(!new_ground_points_.empty())
  {
    if(live_ground_points_.size() == 3)
    {
      live_ground_points_.pop_front();
    }
    live_ground_points_.push_back(new_ground_points_);
  }

  if(live_ground_points_.front().front().x() < new_ground_points_.front().x())
  {
    size_t idx_start = 0;
    while(idx_start < historic_points_.size() && new_ground_points_.front().x() > historic_points_[idx_start].x())
    {
      ++ idx_start;
    }
    historic_points_.erase(historic_points_.begin() + idx_start, historic_points_.end());
    for(const auto & points: live_ground_points_)
    {
      historic_points_.insert(historic_points_.end(), points.begin(), points.end());
    }
    historic_points_.insert(historic_points_.end(), new_ground_points_.begin(), new_ground_points_.end());
    std::sort(historic_points_.begin(), historic_points_.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });
  }

  ground_points_ = historic_points_;
  for(const auto & points: live_ground_points_)
  {
    ground_points_.insert(ground_points_.end(), points.begin(), points.end());
  }

  std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud);
  pc->points_ = ground_points_;
  pc = pc->VoxelDownSample(0.002);
  ground_points_ = pc->points_;
  std::sort(ground_points_.begin(), ground_points_.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });

  auto stop_computation = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
  mc_rtc::log::info("Took {} ms for [E V E R Y T H I N G]", static_cast<double>(duration.count()) / 1000.);

  {
    ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
    const auto & points = pc->points_;
    data_->ground_points.resize(points.size());
    for(size_t i = 0; i < points.size(); ++i)
    {
      const auto & p = points[i];
      data_->ground_points[i] = Eigen::Vector3d(p.x(), X_0_b_.translation().y(), p.z());
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
