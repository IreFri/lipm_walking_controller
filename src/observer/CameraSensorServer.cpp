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

  auto applyKernel = [this](const std::array<float, 2> & pixel, const rs2::depth_frame & frame) -> float
  {
    const float pixel_depth = frame.get_distance(static_cast<int>(pixel[0]), static_cast<int>(pixel[1]));
    float sum_depth = 0.f;
    float counter = 0.f;
    const int half_kernel_size = static_cast<int>(kernel_size_ / 2);
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

  auto angleBetweenVecs = [](const Eigen::Vector3d& v0, const Eigen::Vector3d& v1)
  {
    return std::atan2(v0.cross(v1).norm(), v0.dot(v1));
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
    const int offset = 35;
    for(size_t i = half_width + half_kernel_size - offset; i < width - half_kernel_size - offset; ++i)
    {
      pixel[0] = static_cast<float>(i);
      pixel[1] = static_cast<float>(half_height);

      const float depth = applyKernel(pixel, frame);
      Eigen::Vector3d point = depthToPoint(intrinsics, pixel, depth);
      if(point.z() != 0)
      {
        point.y() = 0.;
        points.push_back(point);
      }
    }

    mc_rtc::log::info("{} / {} points -> {} zeros", points.size(), (width - half_kernel_size - offset) - (half_width + half_kernel_size - offset),
      (width - half_kernel_size - offset) - (half_width + half_kernel_size - offset) - points.size());

    const size_t threshold_nr_zero_to_discard = 45;
    if((width - half_kernel_size - offset) - (half_width + half_kernel_size - offset) - points.size() > threshold_nr_zero_to_discard)
    {
      points.clear();
    }

    // Sort
    std::sort(points.begin(), points.end(),
              [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });

    {
      const std::lock_guard<std::mutex> lock(points_mtx_);
      points_ = points;
    }

    {
      ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
      data_->points.resize(points.size());
      for(size_t i = 0; i < points.size(); ++i)
      {
        data_->points[i] = points[i];
      }
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
  // if(data_->skip)
  // {
  //   return;
  // }

  {
    ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
    if(data_->reset_ground)
    {
      ground_points_.clear();
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

  {
    mc_rtc::log::info("[Step 0] Use previous_pitch and previous_t_z");
    auto start = std::chrono::high_resolution_clock::now();

    pre_new_ground_points_.clear();
    pre_new_camera_points_.clear();
    const sva::PTransformd X_b_b(sva::RotY(previous_pitch_).cast<double>(), Eigen::Vector3d(0., 0., previous_t_z_));

    for(int i = new_camera_points_.size() - 1; i >= 0; --i)
    {
      // From camera frame to world frame
      const sva::PTransformd X_s_p(new_camera_points_[i]);
      const sva::PTransformd X_0_p = X_s_p * data_->X_b_s * X_b_b * data_->X_0_b;
      pre_new_camera_points_.push_back(X_s_p.translation());
      pre_new_ground_points_.push_back(X_0_p.translation());
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::warning("[Step 0] It tooks {} microseconds -> {} milliseconds", duration.count(),
                         duration.count() / 1000.);
  }

  std::vector<Eigen::Vector3d> selected_points;
  std::vector<Eigen::Vector3d> selected_line;
  {
    mc_rtc::log::info("[Step 1] Extract ground component");
    auto start = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<Eigen::Vector3d>> lines;
    lines.push_back(std::vector<Eigen::Vector3d>{});

    std::vector<std::vector<Eigen::Vector3d>> points;
    points.push_back(std::vector<Eigen::Vector3d>{});
    for(size_t i = 1; i < pre_new_ground_points_.size(); ++i)
    {
      const auto & p_0 = pre_new_ground_points_[i - 1];
      const auto & p_1 = pre_new_ground_points_[i];
      const double gradient = (p_1.z() - p_0.z()) / (p_1.x() - p_0.x());
      if(std::abs(gradient) > 1. || (p_1.x() - p_0.x()) > 0.02)
      {
        lines.push_back(std::vector<Eigen::Vector3d>{});
        points.push_back(std::vector<Eigen::Vector3d>{});
      }
      lines.back().push_back(pre_new_ground_points_[i]);
      points.back().push_back(pre_new_camera_points_[i]);
    }

    for(size_t i = 0; i < lines.size(); ++i)
    {
      if(!lines[i].empty())
      {
        const auto & p_0 = lines[i].front();
        const auto & p_1 = lines[i].back();
        const double length = (p_1 - p_0).norm();
        if(length > 0.05)
        {
          selected_points.insert(selected_points.end(), points[i].begin(), points[i].end());
          selected_line.insert(selected_line.end(), lines[i].begin(), lines[i].end());
        }
      }
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::warning("[Step 0] It tooks {} microseconds -> {} milliseconds", duration.count(),
                         duration.count() / 1000.);
  }

  if(selected_points.empty())
  {
    data_->result_ready->notify_all();
    return;
  }

  {
    mc_rtc::log::info("[Step 1] Estimate the z - pitch to bring back to 0");
    auto start = std::chrono::high_resolution_clock::now();
    double pitch = 0.;
    double t_z = 0.;

    ceres::Problem problem;
    for(const auto& point: selected_points)
    {
      const sva::PTransformd X_0_p = sva::PTransformd(point) * data_->X_b_s * data_->X_0_b;
      const sva::PTransformd X_s_p(point);
      ceres::CostFunction* cost_function = PitchZCostFunctor::Create(X_s_p, data_->X_0_b, data_->X_b_s);
      problem.AddResidualBlock(cost_function,  new ceres::CauchyLoss(0.5), &pitch, &t_z);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Update
    previous_pitch_ = pitch;
    previous_t_z_ = t_z;

    const sva::PTransformd X_b_b(sva::RotY(pitch).cast<double>(), Eigen::Vector3d(0., 0., t_z));

    // Compute the 3D point in world frame
    {
      const std::lock_guard<std::mutex> lock(points_mtx_);
      data_->aligned_points.reserve(new_camera_points_.size());
      data_->aligned_points.clear();
      new_ground_points_.reserve(new_camera_points_.size());
      new_ground_points_.clear();
      for(const auto& point: new_camera_points_)
      {
        // From camera frame to world frame
        const sva::PTransformd X_s_p(point);
        const sva::PTransformd X_0_p = X_s_p * data_->X_b_s * X_b_b * data_->X_0_b;
        if(X_0_p.translation().z() > - 0.002)
        {
          data_->aligned_points.push_back(X_0_p.translation());
          new_ground_points_.push_back(X_0_p.translation());
        }
      }
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::warning("[Step 1] It tooks {} microseconds -> {} milliseconds", duration.count(),
                         duration.count() / 1000.);
  }

  {
    mc_rtc::log::info("[Step 2] Fuse new estimated ground with global ground");
    auto start = std::chrono::high_resolution_clock::now();


    std::sort(new_ground_points_.begin(), new_ground_points_.end(),
              [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });

    std::vector<Eigen::Vector3d> filtered_new_ground_points;
    std::vector<Eigen::Vector3d> cluster;
    for(size_t i = 1; i < new_ground_points_.size(); ++i)
    {
      const auto & p_0 = new_ground_points_[i - 1];
      const auto & p_1 = new_ground_points_[i];
      const double gradient = (p_1.z() - p_0.z()) / (p_1.x() - p_0.x());
      if((p_1 - p_0).norm() > 0.005)
      {
        if(cluster.size() > 10)
        {
          filtered_new_ground_points.insert(filtered_new_ground_points.end(), cluster.begin(), cluster.end());
        }
        cluster.clear();
      }
      cluster.push_back(new_ground_points_[i]);
    }
    std::cout << std::endl;

    open3d::geometry::PointCloud open3d_pc;
    ground_points_.insert(ground_points_.end(), filtered_new_ground_points.begin(), filtered_new_ground_points.end());
    open3d_pc.points_ = ground_points_;
    ground_points_ = open3d_pc.VoxelDownSample(0.005)->points_;

    std::sort(ground_points_.begin(), ground_points_.end(),
              [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::warning("[Step 2] It tooks {} microseconds -> {} milliseconds", duration.count(),
                         duration.count() / 1000.);
  }

  {
    ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
    const auto & points = ground_points_;
    data_->ground_points.resize(points.size());
    for(size_t i = 0; i < points.size(); ++i)
    {
      data_->ground_points[i] = points[i];
    }

    data_->selected_points.resize(selected_line.size());
    for(size_t i = 0; i < selected_line.size(); ++i)
    {
      data_->selected_points[i] = selected_line[i];
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
