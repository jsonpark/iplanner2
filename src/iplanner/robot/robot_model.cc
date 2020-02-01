#include "iplanner/robot/robot_model.h"

#include <iostream>
#include <iomanip>

namespace iplanner
{
//
// Joint Iterator
//
RobotModel::JointWrapper RobotModel::LinkWrapper::ParentJoint() const
{
  return JointWrapper(robot_model_, robot_model_->parent_joint_[index_]);
}

std::vector<RobotModel::JointWrapper> RobotModel::LinkWrapper::ChildJoints() const
{
  std::vector<JointWrapper> child_joints;

  for (auto child_joint : robot_model_->child_joints_[index_])
    child_joints.push_back(JointWrapper(robot_model_, child_joint));

  return child_joints;
}

//
// Robot Model
//
RobotModel::RobotModel()
{
}

RobotModel::~RobotModel() = default;

void RobotModel::SetName(const std::string& name)
{
  name_ = name;
}

void RobotModel::AddLink(RobotLink&& link)
{
  links_.push_back(std::move(link));
}

void RobotModel::AddJoint(RobotJoint&& joint)
{
  joints_.push_back(std::move(joint));
}

void RobotModel::CreateTreeModel()
{
  parent_joint_.resize(links_.size(), -1);
  child_joints_.resize(links_.size());

  parent_link_.resize(joints_.size(), -1);
  child_link_.resize(joints_.size(), -1);

  std::unordered_map<std::string, int> link_name_to_index;
  for (int i = 0; i < links_.size(); i++)
    link_name_to_index[links_[i].GetName()] = i;

  std::vector<int> indegrees(links_.size(), 0);
  for (int i = 0; i < joints_.size(); i++)
  {
    const auto& joint = joints_[i];

    int parent_link_index = link_name_to_index[joint.GetParentLinkName()];
    int child_link_index = link_name_to_index[joint.GetChildLinkName()];

    indegrees[child_link_index]++;

    child_joints_[parent_link_index].push_back(i);
    parent_joint_[child_link_index] = i;

    parent_link_[i] = parent_link_index;
    child_link_[i] = child_link_index;
  }

  for (int i = 0; i < links_.size(); i++)
  {
    if (indegrees[i] == 0)
    {
      base_index_ = i;
      break;
    }
  }
}

void RobotModel::PrintRobotLinks() const
{
  std::cout << "Robot [" << name_ << "] links:" << std::endl;
  for (int i = 0; i < links_.size(); i++)
    std::cout << std::setw(4) << i << ": " << links_[i].GetName() << std::endl;
}

void RobotModel::PrintRobotJoints() const
{
  std::cout << "Robot [" << name_ << "] joints:" << std::endl;
  for (int i = 0; i < joints_.size(); i++)
    std::cout << std::setw(4) << i << ": " << joints_[i].GetName() << std::endl;
}

RobotModel RobotModel::FixJoints(const std::vector<std::string>& fixed_joints, const std::vector<double>& joint_values)
{
  return Fixator(*this, fixed_joints, joint_values).FixJoints();
}

RobotModel::Fixator::Fixator(RobotModel& robot_model, const std::vector<std::string>& fixed_joints, const std::vector<double>& joint_values)
  : robot_model_(robot_model), fixed_joints_(fixed_joints), joint_values_(joint_values)
{
  for (int i = 0; i < fixed_joints_.size(); i++)
    fixed_joint_values_[fixed_joints_[i]] = joint_values_[i];
}

RobotModel RobotModel::Fixator::FixJoints()
{
  RobotModel new_model;
  new_model_ = &new_model;

  Traverse(robot_model_.base_index_, 0);

  new_model.CreateTreeModel();
  return new_model;
}

void RobotModel::Fixator::Traverse(int link_index, int new_link_index, Affine3d transform)
{
  const auto& link = robot_model_.links_[link_index];

  if (link_index != robot_model_.base_index_)
  {
    int parent_joint_index = robot_model_.parent_joint_[link_index];
    const auto& parent_joint = robot_model_.joints_[parent_joint_index];

    // Active joint
    if (fixed_joint_values_.find(parent_joint.GetName()) == fixed_joint_values_.cend() &&
      !parent_joint.IsFixed())
    {
      // Add joint
      auto new_joint = parent_joint;
      new_joint.SetParentLinkName(new_model_->links_[new_link_index].GetName());
      new_joint.SetOrigin(transform * parent_joint.GetOrigin());
      new_model_->AddJoint(std::move(new_joint));

      // Add link
      new_model_->links_.push_back(robot_model_.links_[link_index]);
      new_link_index = new_model_->links_.size() - 1;

      transform = Affine3d::Identity();
    }

    // Fix joint
    else
    {
      // Merge with parent link
      transform = transform * parent_joint.GetTransform(fixed_joint_values_[parent_joint.GetName()]);
      new_model_->links_[new_link_index].Merge(link, transform);
    }
  }

  else
    new_model_->links_.push_back(robot_model_.links_[link_index]);

  for (int child_joint_index : robot_model_.child_joints_[link_index])
  {
    int child_link_index = robot_model_.child_link_[child_joint_index];
    Traverse(child_link_index, new_link_index, transform);
  }
}
}
