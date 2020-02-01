#ifndef IPLANNER_ROBOT_ROBOT_JOINT_H_
#define IPLANNER_ROBOT_ROBOT_JOINT_H_

#include <string>

#include "iplanner/types.h"

namespace iplanner
{
class RobotLink;

class RobotJoint
{
public:
  enum class Type
  {
    REVOLUTE,
    CONTINUOUS,
    PRISMATIC,
    FIXED,
    FLOATING,
    PLANAR,
    UNDEFINED
  };

  struct Limit
  {
    double lower;
    double upper;
    double effort;
    double velocity;
  };

public:
  RobotJoint();
  ~RobotJoint();

  void SetName(const std::string& name);
  void SetType(const std::string& type);
  void SetParentLinkName(const std::string& link);
  void SetChildLinkName(const std::string& link);
  void SetOrigin(const Affine3d& origin);
  void SetAxis(const Vector3d& axis);
  void SetLimit(double lower, double upper, double effort, double velocity);

  bool IsFixed() const
  {
    return type_ == Type::FIXED;
  }

  const auto& GetName() const
  {
    return name_;
  }

  const auto& GetParentLinkName() const
  {
    return parent_link_name_;
  }

  const auto& GetChildLinkName() const
  {
    return child_link_name_;
  }

  const auto& GetOrigin() const
  {
    return origin_;
  }

  const auto& GetLimit() const
  {
    return limit_;
  }

  // Operations
  Affine3d GetTransform(double joint_value) const;

private:
  std::string name_;
  Type type_ = Type::UNDEFINED;
  std::string parent_link_name_;
  std::string child_link_name_;
  Affine3d origin_;
  Vector3d axis_;
  Limit limit_;
};
}

#endif // IPLANNER_ROBOT_ROBOT_JOINT_H_
