#include "iplanner/plan/planner.h"

#include <iostream>

namespace iplanner
{
Planner::Planner() = default;

Planner::~Planner() = default;

void Planner::SetRobotModel(std::shared_ptr<RobotModel> robot_model)
{
  robot_model_ = robot_model;
}

void Planner::RunAsync()
{
  fk_ = std::make_shared<RobotState>(robot_model_);

  // Run optimization
  thread_future_ = std::async(std::launch::async, &Planner::Run, this);
}

void Planner::Run()
{
  trajectory_ = std::make_shared<Trajectory>(robot_model_->NumJoints(), config_.num_timepoints, config_.time);
  is_running_ = true;

  while (is_running_)
  {
    using namespace std::chrono_literals;

    // Sleep for 1ms 
    std::this_thread::sleep_for(1s);
  }
}

void Planner::Stop()
{
  std::cout << "stop" << std::endl;

  is_running_ = false;
}
}
