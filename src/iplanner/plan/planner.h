#ifndef IPLANNER_PLAN_PLANNER_H_
#define IPLANNER_PLAN_PLANNER_H_

#include "iplanner/robot/robot_model.h"
#include "iplanner/plan/trajectory.h"

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

  void Run();

private:
  struct ConfigType
  {
    int num_timepoints = 2;
    double time = 1.;
  };

  std::shared_ptr<RobotModel> robot_model_;
  ConfigType config_;

  std::vector<RobotState> fk_;
};
}

#endif // IPLANNER_PLAN_PLANNER_H_
