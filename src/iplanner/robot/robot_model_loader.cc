#include "iplanner/robot/robot_model_loader.h"

#include "iplanner/types.h"
#include "iplanner/robot/robot_joint.h"
#include "iplanner/robot/robot_link.h"

namespace iplanner
{
namespace
{
Affine3d GetOriginFromElement(tinyxml2::XMLElement* element)
{
  Affine3d origin;
  origin.setIdentity();

  auto origin_element = element->FirstChildElement("origin");
  if (origin_element != nullptr)
  {
    auto rpy_str = origin_element->Attribute("rpy");
    if (rpy_str != nullptr)
    {
      double r, p, y;
      sscanf(rpy_str, "%lf%lf%lf", &r, &p, &y);
      origin.rotate(Eigen::AngleAxisd(r, Eigen::Vector3d(1.0, 0.0, 0.0)));
      origin.rotate(Eigen::AngleAxisd(p, Eigen::Vector3d(0.0, 1.0, 0.0)));
      origin.rotate(Eigen::AngleAxisd(y, Eigen::Vector3d(0.0, 0.0, 1.0)));
    }

    auto xyz_str = origin_element->Attribute("xyz");
    if (xyz_str != nullptr)
    {
      double x, y, z;
      sscanf(xyz_str, "%lf%lf%lf", &x, &y, &z);
      origin.translate(Eigen::Vector3d(x, y, z));
    }
  }

  return origin;
}
}

RobotModelLoader::RobotModelLoader() = default;

RobotModelLoader::~RobotModelLoader() = default;

void RobotModelLoader::SetPackageDirectory(const std::string& package_directory)
{
  package_directory_ = package_directory;
}

std::shared_ptr<RobotModel> RobotModelLoader::Load(const std::string& filename)
{
  auto full_filename = package_directory_ + directory_char_ + filename;

  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(full_filename.c_str()) != 0)
    return nullptr;


  // Create an empty robot model
  auto robot_model = std::make_shared<RobotModel>();

  auto robot_element = doc.FirstChildElement("robot");
  robot_model->SetName(robot_element->Attribute("name"));

  // Read xml file
  auto link_element = robot_element->FirstChildElement("link");
  while (link_element != 0)
  {
    RobotLink link;

    link.SetName(link_element->Attribute("name"));

    // Inertial
    auto inertial_element = link_element->FirstChildElement("inertial");
    if (inertial_element != 0)
    {
      auto origin = GetOriginFromElement(inertial_element);
      double mass = std::stod(inertial_element->FirstChildElement("mass")->Attribute("value"));

      auto inertia_element = inertial_element->FirstChildElement("inertia");
      Matrix3d inertia;
      inertia(0, 0) = std::stod(inertia_element->Attribute("ixx"));
      inertia(0, 1) = std::stod(inertia_element->Attribute("ixy"));
      inertia(0, 2) = std::stod(inertia_element->Attribute("ixz"));
      inertia(1, 0) = inertia(0, 1);
      inertia(1, 1) = std::stod(inertia_element->Attribute("iyy"));
      inertia(1, 2) = std::stod(inertia_element->Attribute("iyz"));
      inertia(2, 0) = inertia(0, 2);
      inertia(2, 1) = inertia(1, 2);
      inertia(2, 2) = std::stod(inertia_element->Attribute("izz"));

      link.SetInertial(origin, mass, inertia);
    }

    // Visual
    auto visual_element = link_element->FirstChildElement("visual");
    while (visual_element != 0)
    {
      RobotLink::Visual visual;

      visual.origin = GetOriginFromElement(visual_element);
      visual.geometry = GetGeometryFromElement(visual_element);

      auto material_element = visual_element->FirstChildElement("material");
      if (material_element != 0)
      {
        auto color_element = material_element->FirstChildElement("color");
        if (color_element != 0)
        {
          visual.has_color = true;
          sscanf(color_element->Attribute("rgba"), "%lf%lf%lf%lf", &visual.color(0), &visual.color(1), &visual.color(2), &visual.color(3));
        }
        else
          visual.has_color = false;

        auto texture_element = material_element->FirstChildElement("texture");
        if (texture_element != 0)
        {
          visual.has_texture = true;
          visual.texture_filename = texture_element->Attribute("filename");
        }
        else
          visual.has_texture = false;
      }

      link.AddVisual(visual);

      visual_element = visual_element->NextSiblingElement("visual");
    }

    // Collision
    auto collision_element = link_element->FirstChildElement("collision");
    while (collision_element != 0)
    {
      RobotLink::Collision collision;

      collision.origin = GetOriginFromElement(collision_element);
      collision.geometry = GetGeometryFromElement(collision_element);

      link.AddCollision(collision);

      collision_element = collision_element->NextSiblingElement("collision");
    }

    robot_model->AddLink(std::move(link));

    link_element = link_element->NextSiblingElement("link");
  }

  auto joint_element = robot_element->FirstChildElement("joint");
  while (joint_element != 0)
  {
    RobotJoint joint;

    joint.SetName(joint_element->Attribute("name"));
    joint.SetType(joint_element->Attribute("type"));
    joint.SetParentLinkName(joint_element->FirstChildElement("parent")->Attribute("link"));
    joint.SetChildLinkName(joint_element->FirstChildElement("child")->Attribute("link"));

    auto origin = GetOriginFromElement(joint_element);
    joint.SetOrigin(origin);

    Vector3d axis(1., 0., 0.);
    auto axis_element = joint_element->FirstChildElement("axis");
    if (axis_element != 0)
      sscanf(axis_element->Attribute("xyz"), "%lf%lf%lf", &axis(0), &axis(1), &axis(2));
    joint.SetAxis(axis);

    auto limit_element = joint_element->FirstChildElement("limit");
    if (limit_element)
    {
      double lower = 0., upper = 0., effort, velocity;

      auto lower_str = limit_element->Attribute("lower");
      if (lower_str)
        lower = std::stod(lower_str);

      auto upper_str = limit_element->Attribute("upper");
      if (upper_str)
        upper = std::stod(upper_str);

      effort = std::stod(limit_element->Attribute("effort"));
      velocity = std::stod(limit_element->Attribute("velocity"));

      joint.SetLimit(lower, upper, effort, velocity);
    }

    robot_model->AddJoint(std::move(joint));

    joint_element = joint_element->NextSiblingElement("joint");
  }

  robot_model->CreateTreeModel();

  return robot_model;
}

RobotLink::Geometry RobotModelLoader::GetGeometryFromElement(tinyxml2::XMLElement* element)
{
  RobotLink::Geometry geometry;

  auto geometry_element = element->FirstChildElement("geometry");
  auto box_element = geometry_element->FirstChildElement("box");
  if (box_element != 0)
  {
    geometry.type = RobotLink::Geometry::Type::BOX;
    sscanf(box_element->Attribute("size"), "%lf%lf%lf", &geometry.size.x, &geometry.size.y, &geometry.size.z);
  }

  auto cylinder_element = geometry_element->FirstChildElement("cylinder");
  if (cylinder_element != 0)
  {
    geometry.type = RobotLink::Geometry::Type::CYLINDER;
    geometry.size.radius = std::stod(cylinder_element->Attribute("radius"));
    geometry.size.length = std::stod(cylinder_element->Attribute("length"));
  }

  auto sphere_element = geometry_element->FirstChildElement("sphere");
  if (sphere_element != 0)
  {
    geometry.type = RobotLink::Geometry::Type::SPHERE;
    geometry.size.radius = std::stod(sphere_element->Attribute("radius"));
  }

  auto mesh_element = geometry_element->FirstChildElement("mesh");
  if (mesh_element != 0)
  {
    geometry.type = RobotLink::Geometry::Type::MESH;
    geometry.mesh_filename = ConvertFilenamePackageDirectory(mesh_element->Attribute("filename"));

    auto scale_str = mesh_element->Attribute("scale");
    if (scale_str)
      geometry.size.scale = std::stod(scale_str);
  }

  return geometry;
}

std::string RobotModelLoader::ConvertFilenamePackageDirectory(const std::string& filename)
{
  if (filename.length() >= 10 && filename.substr(0, 10) == "package://")
    return package_directory_ + directory_char_ + filename.substr(10);

  return filename;
}
}
