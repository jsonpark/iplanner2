#ifndef IPLANNER_ROBOT_ROBOT_STATE_H_
#define IPLANNER_ROBOT_ROBOT_STATE_H_

#include "iplanner/robot/robot_model.h"

namespace iplanner
{
class RobotState
{
public:
  RobotState() = delete;

  explicit RobotState(std::shared_ptr<RobotModel> robot_model);

  ~RobotState();

  auto JointPosition(int index) const
  {
    return position_[index];
  }

  auto& JointPosition(int index)
  {
    return position_[index];
  }

  auto NumJoints() const
  {
    return position_.size();
  }

  const auto& JointName(int index) const
  {
    return robot_model_->joints_[index].GetName();
  }

  void ForwardKinematics(const Affine3d& base_transform = Affine3d::Identity());

  const auto& GetTransform(int index) const
  {
    return fk_transforms_[index];
  }

private:
  std::shared_ptr<RobotModel> robot_model_;
  std::vector<double> position_;

  // Forward kinematics
  Affine3dVector fk_transforms_;
};
}

#endif // IPLANNER_ROBOT_ROBOT_STATE_H_
