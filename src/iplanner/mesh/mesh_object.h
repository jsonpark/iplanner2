#ifndef IPLANNER_MESH_MESH_OBJECT_H_
#define IPLANNER_MESH_MESH_OBJECT_H_

#include <memory>

#include "iplanner/object/buffer.h"
#include "iplanner/object/vertex_array.h"

namespace iplanner
{
class MeshObject
{
public:
  MeshObject();

  explicit MeshObject(const std::string& filename);

  ~MeshObject();

  // Copy constructor
  MeshObject(const MeshObject& rhs) = delete;

  MeshObject& operator = (const MeshObject& rhs) = delete;

  // Move constructor
  MeshObject(MeshObject&& rhs) = default;

  MeshObject& operator = (MeshObject&& rhs) = default;

  void Draw();

private:
  Buffer<float> vertices_;
  Buffer<float> normals_;
  Buffer<float> tex_coords_;
  ElementBuffer elements_;

  std::shared_ptr<Texture> texture_;
  VertexArray vao_;
};
}

#endif // IPLANNER_MESH_MESH_OBJECT_H_
