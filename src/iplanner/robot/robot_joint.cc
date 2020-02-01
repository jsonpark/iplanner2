#include "iplanner/robot/robot_joint.h"

namespace iplanner
{
RobotJoint::RobotJoint() = default;

RobotJoint::~RobotJoint() = default;

void RobotJoint::SetName(const std::string& name)
{
  name_ = name;
}

void RobotJoint::SetType(const std::string& type)
{
  if (type == "revolute")
    type_ = Type::REVOLUTE;

  else if (type == "continuous")
    type_ = Type::CONTINUOUS;

  else if (type == "prismatic")
    type_ = Type::PRISMATIC;

  else if (type == "fixed")
    type_ = Type::FIXED;

  else if (type == "floating")
    type_ = Type::FLOATING;

  else if (type == "planar")
    type_ = Type::PLANAR;

  else
    type_ = Type::UNDEFINED;
}

void RobotJoint::SetParentLinkName(const std::string& link)
{
  parent_link_name_ = link;
}

void RobotJoint::SetChildLinkName(const std::string& link)
{
  child_link_name_ = link;
}

void RobotJoint::SetOrigin(const Affine3d& origin)
{
  origin_ = origin;
}

void RobotJoint::SetAxis(const Vector3d& axis)
{
  axis_ = axis;
}

void RobotJoint::SetLimit(double lower, double upper, double effort, double velocity)
{
  limit_.lower = lower;
  limit_.upper = upper;
  limit_.effort = effort;
  limit_.velocity  = velocity;
}

Affine3d RobotJoint::GetTransform(double joint_value) const
{
  Affine3d t = origin_;

  switch (type_)
  {
  case Type::FIXED:
    break;

  case Type::CONTINUOUS:
  case Type::REVOLUTE:
    t.rotate(AngleAxisd(joint_value, axis_));
    break;

  case Type::PRISMATIC:
    t.translate(axis_ * joint_value);
    break;

  default:
    break;
  }

  return t;
}
}
