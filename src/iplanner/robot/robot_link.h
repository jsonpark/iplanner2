#ifndef IPLANNER_ROBOT_ROBOT_LINK_H_
#define IPLANNER_ROBOT_ROBOT_LINK_H_

#include "iplanner/types.h"

namespace iplanner
{
class RobotJoint;

class RobotLink
{
public:
  struct Inertial
  {
    Affine3d origin;
    double mass;
    Matrix3d inertia;
  };

  struct Geometry
  {
    enum class Type
    {
      BOX,
      CYLINDER,
      SPHERE,
      MESH,
      UNDEFINED,
    };

    Type type = Type::UNDEFINED;

    union
    {
      struct
      {
        double x, y, z;
      };
      struct
      {
        double radius;
        double length;
      };
      double scale;
    } size;

    std::string mesh_filename;
  };

  struct Visual
  {
    Affine3d origin;
    Geometry geometry;

    bool has_color;
    Vector4d color;

    bool has_texture;
    std::string texture_filename;
  };

  struct Collision
  {
    Affine3d origin;
    Geometry geometry;
  };

public:
  RobotLink();
  ~RobotLink();

  void SetName(const std::string& name);
  void SetInertial(const Affine3d& origin, double mass, const Matrix3d& inertia);
  void AddVisual(const Visual& visual);
  void AddCollision(const Collision& collision);

  void Merge(const RobotLink& link, Affine3d transform);

  const auto& GetName() const
  {
    return name_;
  }

  const auto& GetVisuals() const
  {
    return visuals_;
  }

private:
  std::string name_;
  Inertial inertial_;
  std::vector<Visual> visuals_;
  std::vector<Collision> collisions_;
};
}

#endif // IPLANNER_ROBOT_ROBOT_LINK_H_
