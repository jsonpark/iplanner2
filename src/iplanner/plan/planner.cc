#include "iplanner/plan/planner.h"

namespace iplanner
{
Planner::Planner() = default;

Planner::~Planner() = default;

void Planner::SetRobotModel(std::shared_ptr<RobotModel> robot_model)
{
  robot_model_ = robot_model;
}

void Planner::Run()
{
  fk_ = std::make_shared<RobotState>(robot_model_);

  // TODO
}
}
