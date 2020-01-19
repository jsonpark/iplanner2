#ifndef IPLANNER_RENDERER_H_
#define IPLANNER_RENDERER_H_

#include "iplanner/object/buffer.h"
#include "iplanner/object/vertex_array.h"
#include "iplanner/object/texture.h"
#include "iplanner/shader/program.h"
#include "iplanner/object/framebuffer.h"
#include "iplanner/scene/scene.h"
#include "iplanner/object/uniform_buffer.h"
#include "iplanner/mesh/mesh.h"
#include "iplanner/mesh/mesh_object.h"
#include "iplanner/human/human_label.h"
#include "iplanner/scene/human_label_node.h"
#include "iplanner/scene/point_cloud_node.h"

struct GLFWwindow;

namespace iplanner
{
class Engine;

class Renderer
{
private:
  static const int max_num_lights_ = 8;

public:
  Renderer() = delete;
  Renderer(Engine* engine);
  ~Renderer();

  void Initialize();
  void Render();

  void SaveImages(const std::string& directory, const std::string& filename_prefix);

  void SetScene(std::shared_ptr<Scene> scene);

  void CreateEmptyTexture(const std::string& name, int width, int height, Texture::Usage usage = Texture::Usage::TEXTURE);

  template<typename T>
  void UpdateTexture(const std::string& name, const std::vector<T>& buffer)
  {
    auto it = textures_.find(name);
    if (it == textures_.cend())
      return;
    it->second->Update((const void*)buffer.data());
  }

private:
  Engine* engine_;

  std::shared_ptr<Scene> scene_;

  void UpdateCameraUniformFromScene();
  void UpdateLightUniformFromScene();

  void TraverseSceneNode(std::shared_ptr<SceneNode> node, Affine3d transform = Affine3d::Identity());

  void DrawMeshesColor();
  void DrawMeshesDepth();

  // Assuming only one point cloud in the scene
  void UpdatePointCloudBuffers();
  void DrawPointCloudColor();
  void DrawPointCloudDepth();

  // Assuming only one human label in the scene
  void UpdateHumanLabelBuffers();
  void DrawHumanLabelColor();
  void DrawHumanLabelDepth();

  std::shared_ptr<Program> program_color_camera_;
  std::shared_ptr<Program> program_depth_camera_;

  UniformBuffer view_camera_uniform_;
  UniformBuffer color_camera_uniform_;
  UniformBuffer depth_camera_uniform_;
  UniformBuffer lights_uniform_;

  // Color framebuffer
  Framebuffer framebuffer_color_{ 1920, 1080 };
  std::shared_ptr<Texture> framebuffer_color_texture_;

  // Depth framebuffer
  Framebuffer framebuffer_depth_{ 512, 424 };
  std::shared_ptr<Texture> framebuffer_depth_texture_;

  Buffer<float> framebuffer_vertices_
  {
    -1.f, -1.f, 0.f, 0.f,
    1.f, -1.f, 1.f, 0.f,
    -1.f, 1.f, 0.f, 1.f,
    1.f, 1.f, 1.f, 1.f
  };

  ElementBuffer framebuffer_elements_{ 0, 1, 2, 3 };

  VertexArray framebuffer_vao_;

  // Depth data vao
  Buffer<float> data_depth_vertices_
  {
    -1.f, -1.f, 1.f, 0.f,
    1.f, -1.f, 1.f, 1.f,
    -1.f, 1.f, 0.f, 0.f,
    1.f, 1.f, 0.f, 1.f
  };

  ElementBuffer data_depth_elements_{ 0, 1, 2, 3 };

  VertexArray data_depth_vao_;

  std::shared_ptr<Program> program_color_screen_;
  std::shared_ptr<Program> program_depth_screen_;

  // Loaded meshes
  std::unordered_map<std::string, MeshObject> mesh_objects_;

  // Named textures
  std::unordered_map<std::string, std::shared_ptr<Texture>> textures_;

  // Ground
  Buffer<float> ground_vertices_
  {
    -10.f, -10.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
     10.f, -10.f, 0.f, 0.f, 0.f, 1.f, 1.f, 0.f,
    -10.f,  10.f, 0.f, 0.f, 0.f, 1.f, 0.f, 1.f,
     10.f,  10.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f,
  };
  ElementBuffer ground_elements_{ 0, 1, 2, 3 };
  VertexArray ground_vao_;

  // Point cloud
  BufferBase<float, BufferType::ARRAY_BUFFER, BufferUsage::DYNAMIC_DRAW> point_cloud_buffer_;
  VertexArray point_cloud_vao_;

  std::shared_ptr<PointCloudNode> point_cloud_node_;
  std::shared_ptr<PointCloud> point_cloud_;
  Affine3d point_cloud_transform_;

  std::shared_ptr<Program> program_color_point_cloud_;
  std::shared_ptr<Program> program_depth_point_cloud_;

  // Human labels to draw
  std::shared_ptr<HumanLabelNode> human_label_node_;
  std::shared_ptr<HumanLabel> human_label_;
  Affine3d human_label_transform_;

  BufferBase<float, BufferType::ARRAY_BUFFER, BufferUsage::DYNAMIC_DRAW> human_label_buffer_;
  ElementBuffer human_label_edge_elements_;
  VertexArray human_label_point_vao_;
  VertexArray human_label_edge_vao_;

  std::shared_ptr<Program> program_human_edge_;

  // Mesh files to draw
  std::vector<std::pair<std::string, Affine3d>> meshes_to_draw_;
};
}

#endif // IPLANNER_RENDERER_H_
