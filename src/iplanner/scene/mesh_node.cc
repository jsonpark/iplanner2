#include "iplanner/scene/mesh_node.h"

namespace iplanner
{
MeshNode::MeshNode(const std::string& filename)
  : filename_(filename)
{
}

MeshNode::~MeshNode() = default;
}
