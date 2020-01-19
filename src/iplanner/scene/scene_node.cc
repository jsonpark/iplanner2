#include "iplanner/scene/scene_node.h"

namespace iplanner
{
void SceneNode::Connect(std::shared_ptr<SceneNode> parent, std::shared_ptr<SceneNode> child)
{
  parent->AddChild(child);
  child->SetParent(parent);
}

SceneNode::SceneNode() = default;

SceneNode::~SceneNode() = default;
}
