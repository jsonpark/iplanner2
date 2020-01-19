#ifndef IPLANNER_SCENE_MESH_NODE_H_
#define IPLANNER_SCENE_MESH_NODE_H_

#include "iplanner/scene/scene_node.h"

#include "iplanner/types.h"

namespace iplanner
{
class MeshNode : public SceneNode
{
public:
  MeshNode() = delete;

  explicit MeshNode(const std::string& filename);

  ~MeshNode();

  bool IsMeshNode() const override
  {
    return true;
  }

  const auto& GetMeshFilename() const
  {
    return filename_;
  }

private:
  std::string filename_;
};
}

#endif // IPLANNER_SCENE_MESH_NODE_H_
