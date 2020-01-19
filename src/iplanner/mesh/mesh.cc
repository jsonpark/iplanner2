#include "iplanner/mesh/mesh.h"

namespace iplanner
{
Mesh::Mesh() = default;

Mesh::~Mesh() = default;

void Mesh::AddVertex(float x, float y, float z)
{
  vertices_.push_back(x);
  vertices_.push_back(y);
  vertices_.push_back(z);
}

void Mesh::AddNormal(float x, float y, float z)
{
  normals_.push_back(x);
  normals_.push_back(y);
  normals_.push_back(z);
}

void Mesh::AddTexCoord(float u, float v)
{
  tex_coords_.push_back(u);
  tex_coords_.push_back(v);
}

void Mesh::AddIndex(int i)
{
  indices_.push_back(i);
}

void Mesh::SetDiffuseTextureFilename(const std::string& filename)
{
  has_diffuse_texture_ = true;
  diffuse_texture_filename_ = filename;
}
}
