#ifndef IPLANNER_SCENE_ROBOT_NODE_H_
#define IPLANNER_SCENE_ROBOT_NODE_H_

#include "iplanner/scene/scene_node.h"

#include "iplanner/robot/robot_model.h"

namespace iplanner
{
class RobotNode : public SceneNode
{
public:
  static std::shared_ptr<RobotNode> Create(std::shared_ptr<RobotModel> robot_model);

private:
  static void Traverse(std::shared_ptr<RobotNode> root, std::shared_ptr<SceneNode> node, RobotModel::LinkWrapper link);

public:
  RobotNode();
  ~RobotNode();

  void SetJointValue(const std::string& joint_name, double v);

private:
  std::shared_ptr<RobotModel> robot_model_;

  std::unordered_map<std::string, RobotModel::JointWrapper> joint_name_to_joint_;
  std::unordered_map<std::string, std::shared_ptr<SceneNode>> joint_name_to_node_;
};
}

#endif // IPLANNER_SCENE_ROBOT_NODE_H_
