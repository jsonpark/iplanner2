#include "iplanner/mesh/mesh_object.h"

#include <iostream>

#include "iplanner/mesh/mesh.h"
#include "iplanner/mesh/mesh_loader.h"

namespace iplanner
{
MeshObject::MeshObject() = default;

MeshObject::MeshObject(const std::string& filename)
{
  MeshLoader mesh_loader;
  auto mesh = mesh_loader.Load(filename);
  vertices_.CopyFrom(mesh->GetVertices());
  normals_.CopyFrom(mesh->GetNormals());
  if (mesh->HasDiffuseTexture())
    tex_coords_.CopyFrom(mesh->GetTexCoords());
  elements_.CopyFrom(mesh->GetIndices());

  vao_.BufferPointer(0, 3, vertices_);
  vao_.BufferPointer(1, 3, normals_);
  if (mesh->HasDiffuseTexture())
    vao_.BufferPointer(2, 2, tex_coords_);
  vao_.SetDrawElementMode(VertexArray::DrawMode::TRIANGLES, elements_);

  if (mesh->HasDiffuseTexture())
  {
    texture_ = std::make_shared<Texture>(mesh->GetDiffuseTextureFilename());
    vao_.SetTexture(0, texture_);
  }
}

MeshObject::~MeshObject() = default;

void MeshObject::Draw()
{
  vao_.Draw();
}
}
