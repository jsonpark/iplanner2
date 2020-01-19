#ifndef IPLANNER_SCENE_HUMAN_LABEL_NODE_H_
#define IPLANNER_SCENE_HUMAN_LABEL_NODE_H_

#include "iplanner/scene/scene_node.h"

#include "iplanner/types.h"
#include "iplanner/human/human_label.h"

namespace iplanner
{
class HumanLabelNode : public SceneNode
{
private:
  using Edge = std::pair<int, int>;

public:
  HumanLabelNode();

  ~HumanLabelNode();

  bool IsHumanLabelNode() const override
  {
    return true;
  }

  void SetLabel(std::shared_ptr<HumanLabel> label);

  std::shared_ptr<HumanLabel> GetLabel() const;

  void AddEdge(int i, int j)
  {
    edges_.push_back(std::make_pair(i, j));
  }

  const auto& GetEdges() const
  {
    return edges_;
  }

private:
  std::shared_ptr<HumanLabel> label_;

  std::vector<Edge> edges_;
};
}

#endif // IPLANNER_SCENE_HUMAN_LABEL_NODE_H_
