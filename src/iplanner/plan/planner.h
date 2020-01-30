#ifndef IPLANNER_PLAN_PLANNER_H_
#define IPLANNER_PLAN_PLANNER_H_

#include "iplanner/robot/robot_model.h"
#include "iplanner/robot/robot_state.h"
#include "iplanner/plan/trajectory.h"

#include <atomic>
#include <mutex>
#include <future>

namespace iplanner
{
class Planner
{
public:
  Planner();
  ~Planner();

  void SetRobotModel(std::shared_ptr<RobotModel> robot_model);

  auto& Config()
  {
    return config_;
  }

  const auto& Config() const
  {
    return config_;
  }

  void RunAsync();
  void Stop();

  Trajectory GetTrajectory()
  {
    std::scoped_lock<std::mutex> lock(trajectory_mutex_);
    return *trajectory_;
  }

private:
  struct ConfigType
  {
    int num_timepoints = 2;
    double time = 1.;
    double prediction_timestep = 0.5;
  };

  void Run();

  std::shared_ptr<RobotModel> robot_model_;
  ConfigType config_;

  std::shared_ptr<RobotState> fk_;

  std::atomic_bool is_running_;

  std::mutex trajectory_mutex_;
  std::shared_ptr<Trajectory> trajectory_;

  std::future<void> thread_future_;
};
}

#endif // IPLANNER_PLAN_PLANNER_H_
