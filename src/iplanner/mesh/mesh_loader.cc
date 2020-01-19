#include "iplanner/mesh/mesh_loader.h"

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

namespace iplanner
{
MeshLoader::MeshLoader() = default;

MeshLoader::~MeshLoader() = default;

std::shared_ptr<Mesh> MeshLoader::Load(const std::string& filename)
{
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(filename.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals);
  if (scene == NULL)
    return nullptr;

  auto mesh = std::make_shared<Mesh>();

  aiNode* scene_node = scene->mRootNode;

  // Assume only one mesh is contained in the file
  aiMesh* ai_mesh = scene->mMeshes[0];

  for (int i = 0; i < ai_mesh->mNumVertices; i++)
    mesh->AddVertex(ai_mesh->mVertices[i].x, ai_mesh->mVertices[i].y, ai_mesh->mVertices[i].z);

  if (ai_mesh->mNormals != NULL)
  {
    for (int i = 0; i < ai_mesh->mNumVertices; i++)
      mesh->AddNormal(ai_mesh->mNormals[i].x, ai_mesh->mNormals[i].y, ai_mesh->mNormals[i].z);
  }

  if (ai_mesh->mTextureCoords[0] != NULL)
  {
    for (int i = 0; i < ai_mesh->mNumVertices; i++)
      mesh->AddTexCoord(ai_mesh->mTextureCoords[0][i].x, ai_mesh->mTextureCoords[0][i].y);
  }

  for (int i = 0; i < ai_mesh->mNumFaces; i++)
  {
    aiFace face = ai_mesh->mFaces[i];
    for (int j = 0; j < face.mNumIndices; j++)
      mesh->AddIndex(face.mIndices[j]);
  }

  aiMaterial* material = scene->mMaterials[ai_mesh->mMaterialIndex];
  int num_diffuse_textures = material->GetTextureCount(aiTextureType_DIFFUSE);
  for (int i = 0; i < num_diffuse_textures; i++)
  {
    aiString str;
    material->GetTexture(aiTextureType_DIFFUSE, i, &str);
    std::string texture_filename = str.C_Str();

    // Make absolute path
    texture_filename = filename.substr(0, filename.find_last_of("\\/") + 1) + texture_filename;

    mesh->SetDiffuseTextureFilename(texture_filename);
  }

  return mesh;
}
}
