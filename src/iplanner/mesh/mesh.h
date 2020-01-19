#ifndef IPLANNER_MESH_MESH_H_
#define IPLANNER_MESH_MESH_H_

#include <vector>
#include <string>

namespace iplanner
{
class Mesh
{
public:
  Mesh();
  ~Mesh();

  void AddVertex(float x, float y, float z);
  void AddNormal(float x, float y, float z);
  void AddTexCoord(float u, float v);
  void AddIndex(int i);

  void SetDiffuseTextureFilename(const std::string& filename);

  const auto& GetVertices() const
  {
    return vertices_;
  }

  const auto& GetNormals() const
  {
    return normals_;
  }

  const auto& GetTexCoords() const
  {
    return tex_coords_;
  }

  const auto& GetIndices() const
  {
    return indices_;
  }

  bool HasDiffuseTexture() const noexcept
  {
    return has_diffuse_texture_;
  }

  const auto& GetDiffuseTextureFilename() const noexcept
  {
    return diffuse_texture_filename_;
  }

private:
  std::vector<float> vertices_;
  std::vector<float> normals_;
  std::vector<float> tex_coords_;
  std::vector<unsigned int> indices_;

  bool has_diffuse_texture_ = false;
  std::string diffuse_texture_filename_;
};
}

#endif // IPLANNER_MESH_MESH_H_
