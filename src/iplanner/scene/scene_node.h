#ifndef IPLANNER_SCENE_SCENE_NODE_H_
#define IPLANNER_SCENE_SCENE_NODE_H_

#include "iplanner/types.h"

namespace iplanner
{
class SceneNode
{
public:
  static void Connect(std::shared_ptr<SceneNode> parent, std::shared_ptr<SceneNode> child);

public:
  SceneNode();
  ~SceneNode();

  virtual bool IsMeshNode() const
  {
    return false;
  }

  virtual bool IsGroundNode() const
  {
    return false;
  }

  virtual bool IsPointCloudNode() const
  {
    return false;
  }

  virtual bool IsHumanLabelNode() const
  {
    return false;
  }

  bool IsShown() const noexcept
  {
    return is_shown_;
  }

  void Show() noexcept
  {
    is_shown_ = true;
  }

  void Hide() noexcept
  {
    is_shown_ = false;
  }

  const auto& GetTransform() const
  {
    return transform_;
  }

  void SetTransform(const Affine3d& transform)
  {
    transform_ = transform;
  }

  void SetParent(std::shared_ptr<SceneNode> parent)
  {
    parent_ = parent;
  }

  auto GetParent() const
  {
    return parent_.lock();
  }

  void AddChild(std::shared_ptr<SceneNode> child)
  {
    children_.push_back(child);
  }

  const auto& GetChildren() const
  {
    return children_;
  }

private:
  Affine3d transform_ = Affine3d::Identity();

  bool is_shown_ = true;

  std::weak_ptr<SceneNode> parent_;
  std::vector<std::shared_ptr<SceneNode>> children_;
};
}

#endif // IPLANNER_SCENE_SCENE_NODE_H_
