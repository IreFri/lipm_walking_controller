#pragma once

#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
namespace ipc = boost::interprocess;

#include <boost/date_time.hpp>
namespace ptime = boost::posix_time;
using pclock = ptime::microsec_clock;

#include <sys/types.h>
#include <unistd.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace lipm_walking
{

/** Unique ID for all CameraSensor SHM */
static constexpr auto CAMERA_SENSOR_SHM_ID = "CameraSensorSHM";

/** Simple wrapper around a process pid to check if it's alive */
struct PID
{
  pid_t pid = -1;

  bool isAlive() const noexcept;
};

/** Shared data between the camera server and the camera observer */
struct CameraSensorShared
{
  /** Sync mechanism */
  ipc::interprocess_mutex mtx;
  ipc::interprocess_condition client_idle;
  ipc::interprocess_condition server_idle;
  ipc::offset_ptr<ipc::interprocess_condition> data_ready;
  ipc::offset_ptr<ipc::interprocess_condition> compute_ready;
  ipc::offset_ptr<ipc::interprocess_condition> result_ready;
  PID client;
  PID server;
  /** Data passed by the client (observer code) */
  /** If true, skip the computation this time */
  bool skip = false;
  /** Sensor normal force */
  double fz = 0.0;
  /** Sensor's body to sensor */
  sva::PTransformd X_b_s;
  /** Sensor's body world pos */
  sva::PTransformd X_0_b;
  /** Output received after the computation */
  using v3d_allocator = ipc::allocator<Eigen::Vector3d, ipc::managed_shared_memory::segment_manager>;
  ipc::interprocess_mutex points_mtx;
  std::vector<Eigen::Vector3d, v3d_allocator> points;

  /** Register a new client (client.isAlive() must be false) */
  void newClient(pid_t pid);

  /** Register a new server (server.isAlive() must be false) */
  void newServer(pid_t pid);

  /** Get the data associated to a given \ref name
   *
   * The data is created if it does not exist yet
   */
  static CameraSensorShared * get(const char * name);

private:
  /** Reset the interprocess_condition that might be in a strange state before a new client/server runs */
  void resetConditions();

  /** Key-idiom to forbid public construction while being compatible with the shared memory constructor */
  struct CtorKey
  {
  };

public:
  CameraSensorShared(CtorKey);
};

} // namespace lipm_walking
