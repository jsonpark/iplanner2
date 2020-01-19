#include "iplanner/robot/robot_link.h"

namespace iplanner
{
RobotLink::RobotLink() = default;

RobotLink::~RobotLink() = default;

void RobotLink::SetName(const std::string& name)
{
  name_ = name;
}

void RobotLink::SetInertial(const Affine3d& origin, double mass, const Matrix3d& inertia)
{
  inertial_.origin = origin;
  inertial_.mass = mass;
  inertial_.inertia = inertia;
}

void RobotLink::AddVisual(const Visual& visual)
{
  visuals_.push_back(visual);
}

void RobotLink::AddCollision(const Collision& collision)
{
  collisions_.push_back(collision);
}

void RobotLink::Merge(const RobotLink& link, Affine3d transform)
{
  inertial_.mass += link.inertial_.mass;
  // TODO: merge inertia matrix and origin

  for (auto link_visual : link.visuals_)
  {
    link_visual.origin = transform * link_visual.origin;
    AddVisual(link_visual);
  }

  for (const auto& link_collision : link.collisions_)
    AddCollision(link_collision);
}
}
