#ifndef IPLANNER_ROBOT_ROBOT_MODEL_H_
#define IPLANNER_ROBOT_ROBOT_MODEL_H_

#include <string>
#include <unordered_map>

#include "iplanner/robot/robot_link.h"
#include "iplanner/robot/robot_joint.h"

namespace iplanner
{
class RobotModel
{
  friend class RobotState;

public:
  class JointWrapper;

  class LinkWrapper
  {
    friend class RobotModel;

  public:
    LinkWrapper() = default;

    LinkWrapper(RobotModel* robot_model, int index)
      : robot_model_(robot_model), index_(index)
    {
    }

    ~LinkWrapper() = default;

    // Pointer operators
    RobotLink* operator -> ()
    {
      return &robot_model_->links_[index_];
    }

    const RobotLink* operator -> () const
    {
      return &robot_model_->links_[index_];
    }

    RobotLink& operator * ()
    {
      return robot_model_->links_[index_];
    }

    const RobotLink& operator * () const
    {
      return robot_model_->links_[index_];
    }

    // Operations
    JointWrapper ParentJoint() const;

    std::vector<RobotModel::JointWrapper> ChildJoints() const;

  private:
    RobotModel* robot_model_ = nullptr;
    int index_ = -1;
  };

  class JointWrapper
  {
    friend class RobotModel;

  public:
    JointWrapper() = default;

    JointWrapper(RobotModel* robot_model, int index)
      : robot_model_(robot_model), index_(index)
    {
    }

    ~JointWrapper() = default;

    // Pointer operators
    RobotJoint* operator -> ()
    {
      return &robot_model_->joints_[index_];
    }

    const RobotJoint* operator -> () const
    {
      return &robot_model_->joints_[index_];
    }

    RobotJoint& operator * ()
    {
      return robot_model_->joints_[index_];
    }

    const RobotJoint& operator * () const
    {
      return robot_model_->joints_[index_];
    }

    // Operations
    auto GetParentLink() const
    {
      return LinkWrapper(robot_model_, robot_model_->parent_link_[index_]);
    }

    auto GetChildLink() const
    {
      return LinkWrapper(robot_model_, robot_model_->child_link_[index_]);
    }

  private:
    RobotModel* robot_model_ = nullptr;
    int index_ = -1;
  };

public:
  RobotModel();
  ~RobotModel();

  void SetName(const std::string& name);
  void AddLink(RobotLink&& link);
  void AddJoint(RobotJoint&& joint);

  void CreateTreeModel();

  auto GetBase()
  {
    return LinkWrapper(this, base_index_);
  }

  RobotModel FixJoints(const std::vector<std::string>& fixed_joints, const std::vector<double>& joint_values);

  // Print infos
  void PrintRobotLinks() const;
  void PrintRobotJoints() const;

private:
  std::string name_;
  std::vector<RobotLink> links_;
  std::vector<RobotJoint> joints_;

  // Tree model
  int base_index_ = -1;

  // Link data
  std::vector<int> parent_joint_;
  std::vector<std::vector<int>> child_joints_;

  // Joint data
  std::vector<int> parent_link_;
  std::vector<int> child_link_;

  // Temporary variables for fixing robot joints
  class Fixator
  {
  public:
    Fixator() = delete;

    Fixator(RobotModel& robot_model, const std::vector<std::string>& fixed_joints, const std::vector<double>& joint_values);

    ~Fixator() = default;

    RobotModel FixJoints();

  private:
    void Traverse(int link_index, int new_link_index, Affine3d transform = Affine3d::Identity());

    RobotModel& robot_model_;
    const std::vector<std::string>& fixed_joints_;
    const std::vector<double>& joint_values_;

    std::unordered_map<std::string, double> fixed_joint_values_;
    RobotModel* new_model_;
  };
};
}

#endif // IPLANNER_ROBOT_ROBOT_MODEL_H_
