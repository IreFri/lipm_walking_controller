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

    const size_t half_height = static_cast<size_t>(static_cast<double>(height) * 0.5);
    const size_t half_width = static_cast<size_t>(static_cast<double>(width) * 0.5);

    std::array<float, 2> pixel;
    const size_t half_kernel_size = kernel_size_ / 2;
    std::vector<Eigen::Vector3d> points;
    const int offset = 20;
    for(size_t i = half_height + half_kernel_size; i < height - half_kernel_size - offset; ++i)
    {
      pixel[0] = static_cast<float>(i);
      pixel[1] = static_cast<float>(half_height);

      const float depth = applyKernel(pixel, frame);
      const Eigen::Vector3d point = depthToPoint(intrinsics, pixel, depth);
      if(point.z() != 0 && point.x() < 0.05 && point.z() < 0.30)
      {
        point.y() = 0.;
        points.push_back(point);
      }
    }

    // Sort
    std::sort(points.begin(), points.end(),
              [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });

    {
      const std::lock_guard<std::mutex> lock(points_mtx_);
      points_ = points;
    }
    if(data_->client.isAlive())
    {
      data_->data_ready->notify_all();
    }
  }
}

void CameraSensorServer::computation()
{
  pc_estimated_ground_points_ = std::make_shared<open3d::geometry::PointCloud>();
  pc_transformed_estimated_ground_points_ = std::make_shared<open3d::geometry::PointCloud>();
  pc_full_ground_reconstructed_points_ = std::make_shared<open3d::geometry::PointCloud>();
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
  {
    const std::lock_guard<std::mutex> lock(points_mtx_);
    ground_points_ = points_;
    corrected_ground_points_.reserve(ground_points_.size());
  }
  {
    mc_rtc::log::info("[Step 1] Estimate the z - pitch to bring back to 0");
    auto start = std::chrono::high_resolution_clock::now();
    double pitch = 0.;
    double t_z = 0.;

    ceres::Problem problem;
    for(const auto & point : ground_points_)
    {
      const sva::PTransformd X_0_p(point);
      const sva::PTransformd X_s_p = X_0_p * (data_->X_b_s * data_->X_0_b).inv();
      ceres::CostFunction * cost_function = PitchZCostFunctor::Create(X_s_p, data_->X_0_b, data_->X_b_s);
      problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), &pitch, &t_z);
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
      corrected_ground_points_.clear();
      for(const auto & pp : ground_points_)
      {
        // From camera frame to world frame
        sva::PTransformd X_s_p(pp);
        sva::PTransformd X_0_p = X_s_p * data_->X_b_s * X_b_b * data_->X_0_b;
        corrected_ground_points_.push_back(X_0_p.translation());
      }
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::warning("[Step 1] It tooks {} microseconds -> {} milliseconds", duration.count(),
                         duration.count() / 1000.);
  }
  {
    mc_rtc::log::info("[Step 2] Perform ICP");
    auto start = std::chrono::high_resolution_clock::now();

    pc_estimated_ground_points_->points_ = corrected_ground_points_;

    if(pc_full_ground_reconstructed_points_->points_.empty())
    {
      *pc_full_ground_reconstructed_points_ = *pc_estimated_ground_points_;
      ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
      data_->points.resize(0);
      return;
    }

    pc_estimated_ground_points_ = pc_estimated_ground_points_->VoxelDownSample(0.005);

    const auto front =
        Eigen::Vector3d(data_->X_0_b.translation().x() + 0.0, data_->X_0_b.translation().y() - 0.05, -0.04);
    const auto back = Eigen::Vector3d(
        Eigen::Vector3d(data_->X_0_b.translation().x() + 0.55, data_->X_0_b.translation().y() + 0.05, 0.04));
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
      *pc_transformed_estimated_ground_points_ = *pc_estimated_ground_points_;
      pc_transformed_estimated_ground_points_->Transform(result.transformation_);
      *pc_full_ground_reconstructed_points_ += *pc_transformed_estimated_ground_points_;
      pc_full_ground_reconstructed_points_ = pc_full_ground_reconstructed_points_->VoxelDownSample(0.005);
      std::sort(pc_full_ground_reconstructed_points_->points_.begin(),
                pc_full_ground_reconstructed_points_->points_.end(),
                [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::warning("[Step 2] It tooks {} microseconds -> {} milliseconds", duration.count(),
                         duration.count() / 1000.);
  }

  {
    ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
    const auto & points = pc_full_ground_reconstructed_points_->points_;
    data_->points.resize(points.size());
    for(size_t i = 0; i < points.size(); ++i)
    {
      data_->points[i] = points[i];
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
