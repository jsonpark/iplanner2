#ifndef IPLANNER_MESH_MESH_LOADER_H_
#define IPLANNER_MESH_MESH_LOADER_H_

#include <memory>

#include "iplanner/mesh/mesh.h"

namespace iplanner
{
class MeshLoader
{
public:
  MeshLoader();

  ~MeshLoader();

  std::shared_ptr<Mesh> Load(const std::string& filename);

private:
};
}

#endif // IPLANNER_MESH_MESH_LOADER_H_
