#ifndef IPLANNER_SCENE_GROUND_NODE_H_
#define IPLANNER_SCENE_GROUND_NODE_H_

#include "iplanner/scene/scene_node.h"

#include "iplanner/types.h"

namespace iplanner
{
class GroundNode : public SceneNode
{
public:
  GroundNode();

  ~GroundNode();

  bool IsGroundNode() const override
  {
    return true;
  }

private:
};
}

#endif // IPLANNER_SCENE_GROUND_NODE_H_
