#include "iplanner/robot/robot_state.h"

namespace iplanner
{
RobotState::RobotState(std::shared_ptr<RobotModel> robot_model)
  : robot_model_(robot_model),
  position_(robot_model_->joints_.size(), 0.),
  fk_transforms_(robot_model_->links_.size())
{
}

RobotState::~RobotState() = default;

void RobotState::ForwardKinematics(const Affine3d& base_transform)
{
  std::vector<int> stack;
  stack.reserve(robot_model_->links_.size());

  int base_link_index = robot_model_->base_index_;
  stack.push_back(base_link_index);
  fk_transforms_[base_link_index] = base_transform;

  for (int i = 0; i < stack.size(); i++)
  {
    int link_index = stack[i];

    if (i > 0)
    {
      int parent_joint_index = robot_model_->parent_joint_[link_index];
      const auto& parent_joint = robot_model_->joints_[parent_joint_index];
      auto joint_value = position_[parent_joint_index];

      int parent_link_index = robot_model_->parent_link_[parent_joint_index];
      const auto& parent_link = robot_model_->links_[parent_link_index];

      fk_transforms_[link_index] = fk_transforms_[parent_link_index] * parent_joint.GetTransform(joint_value);
    }

    for (int j = 0; j < robot_model_->child_joints_[link_index].size(); j++)
    {
      int child_joint_index = robot_model_->child_joints_[link_index][j];
      int child_link_index = robot_model_->child_link_[child_joint_index];

      stack.push_back(child_link_index);
    }
  }
}
}
