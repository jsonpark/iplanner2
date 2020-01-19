#include "iplanner/scene/robot_node.h"

#include "iplanner/scene/mesh_node.h"

namespace iplanner
{
std::shared_ptr<RobotNode> RobotNode::Create(std::shared_ptr<RobotModel> robot_model)
{
  auto base_node = std::make_shared<RobotNode>();

  base_node->robot_model_ = robot_model;

  base_node->joint_name_to_node_["base"] = base_node;

  Traverse(base_node, base_node, robot_model->GetBase());
  return base_node;
}

void RobotNode::Traverse(std::shared_ptr<RobotNode> root, std::shared_ptr<SceneNode> node, RobotModel::LinkWrapper link)
{
  // Add visuals
  const auto& visuals = link->GetVisuals();

  for (int i = 0; i < visuals.size(); i++)
  {
    const auto& visual = visuals[i];

    switch (visual.geometry.type)
    {
    case RobotLink::Geometry::Type::MESH:
    {
      auto visual_node = std::make_shared<MeshNode>(visual.geometry.mesh_filename);
      visual_node->SetTransform(visual.origin);

      node->AddChild(visual_node);
      visual_node->SetParent(node);
    }
      break;

    case RobotLink::Geometry::Type::BOX:
    case RobotLink::Geometry::Type::CYLINDER:
    case RobotLink::Geometry::Type::SPHERE:
      // TODO: primitive geometry
      break;

    default:
      break;
    }
  }

  // Traverse child subtrees
  const auto& joints = link.ChildJoints();

  for (auto joint : joints)
  {
    auto child_link = joint.GetChildLink();

    auto child_link_node = std::make_shared<SceneNode>();
    child_link_node->SetTransform(joint->GetTransform(0.));

    node->AddChild(child_link_node);
    child_link_node->SetParent(node);

    const auto& joint_name = joint->GetName();
    root->joint_name_to_joint_[joint_name] = joint;
    root->joint_name_to_node_[joint_name] = child_link_node;

    Traverse(root, child_link_node, child_link);
  }
}

RobotNode::RobotNode() = default;

RobotNode::~RobotNode() = default;

void RobotNode::SetJointValue(const std::string& joint_name, double v)
{
  if (joint_name_to_node_.find(joint_name) != joint_name_to_node_.cend())
    joint_name_to_node_[joint_name]->SetTransform(joint_name_to_joint_[joint_name]->GetTransform(v));
}
}
