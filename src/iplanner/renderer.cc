#include "iplanner/renderer.h"

#include <iostream>

#include <glad/glad.h>

#include "iplanner/engine.h"
#include "iplanner/mesh/mesh_loader.h"
#include "iplanner/scene/mesh_node.h"

namespace iplanner
{
Renderer::Renderer(Engine* engine)
  : engine_(engine),
  point_cloud_buffer_(512 * 424 * 6),
  human_label_buffer_(25 * 6),
  human_label_edge_elements_(100)
{
}

Renderer::~Renderer()
{
}

void Renderer::Initialize()
{
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LINE_SMOOTH);


  // Uniform block setup
  color_camera_uniform_.AddMember(UniformBuffer::Type::MAT4, "projection");
  color_camera_uniform_.AddMember(UniformBuffer::Type::MAT4, "view");

  depth_camera_uniform_.AddMember(UniformBuffer::Type::MAT4, "projection");
  depth_camera_uniform_.AddMember(UniformBuffer::Type::MAT4, "view");

  view_camera_uniform_.AddMember(UniformBuffer::Type::MAT4, "projection");
  view_camera_uniform_.AddMember(UniformBuffer::Type::MAT4, "view");

  for (int i = 0; i < 8; i++)
  {
    std::string var_name = "lights[" + std::to_string(i) + "]";
    lights_uniform_.AddMember(UniformBuffer::Type::BOOL, var_name + ".use");
    lights_uniform_.AddMember(UniformBuffer::Type::INT, var_name + ".type");
    lights_uniform_.AddMember(UniformBuffer::Type::VEC3, var_name + ".position");
    lights_uniform_.AddMember(UniformBuffer::Type::VEC3, var_name + ".ambient");
    lights_uniform_.AddMember(UniformBuffer::Type::VEC3, var_name + ".diffuse");
    lights_uniform_.AddMember(UniformBuffer::Type::VEC3, var_name + ".specular");
    lights_uniform_.AddMember(UniformBuffer::Type::VEC3, var_name + ".attenuation");
  }


  // Shaders
  program_color_camera_ = std::make_shared<Program>("..\\src\\shader", "color_camera");
  program_color_camera_->BindUniformBuffer(0, color_camera_uniform_);
  program_color_camera_->BindUniformBuffer(1, lights_uniform_);

  program_depth_camera_ = std::make_shared<Program>("..\\src\\shader", "depth_camera");
  program_depth_camera_->BindUniformBuffer(0, depth_camera_uniform_);


  // Framebuffer textures
  framebuffer_color_texture_ = std::make_shared<Texture>(1920, 1080, Texture::Usage::COLOR_FRAMEBUFFER);

  // Store depth values as u32 integers
  framebuffer_depth_texture_ = std::make_shared<Texture>(512, 424, Texture::Usage::U32_FRAMEBUFFER);

  framebuffer_color_.AttachColorTexture(0, framebuffer_color_texture_);
  framebuffer_color_.CreateDepthStencilRenderbuffer();

  framebuffer_depth_.AttachColorTexture(0, framebuffer_depth_texture_);
  framebuffer_depth_.CreateDepthStencilRenderbuffer();

  // Framebuffer screen draw
  framebuffer_vao_.BufferPointer(0, 2, framebuffer_vertices_, 4, 0);
  framebuffer_vao_.BufferPointer(1, 2, framebuffer_vertices_, 4, 2);
  framebuffer_vao_.SetDrawElementMode(VertexArray::DrawMode::TRIANGLE_STRIP, framebuffer_elements_);

  // Data depth image screen draw
  data_depth_vao_.BufferPointer(0, 2, data_depth_vertices_, 4, 0);
  data_depth_vao_.BufferPointer(1, 2, data_depth_vertices_, 4, 2);
  data_depth_vao_.SetDrawElementMode(VertexArray::DrawMode::TRIANGLE_STRIP, data_depth_elements_);

  // Framebuffer screen draw shader
  program_color_screen_ = std::make_shared<Program>("..\\src\\shader", "color_screen");
  program_color_screen_->Uniform1i("tex", 0);

  program_depth_screen_ = std::make_shared<Program>("..\\src\\shader", "depth_screen");
  program_depth_screen_->Uniform1i("depth", 0);

  // Ground vao
  ground_vao_.BufferPointer(0, 3, ground_vertices_, 8, 0);
  ground_vao_.BufferPointer(1, 3, ground_vertices_, 8, 3);
  ground_vao_.BufferPointer(2, 2, ground_vertices_, 8, 6);
  ground_vao_.SetDrawElementMode(VertexArray::DrawMode::TRIANGLE_STRIP, ground_elements_);


  // Point cloud vao
  program_color_point_cloud_ = std::make_shared<Program>("..\\src\\shader", "color_point_cloud");
  program_depth_point_cloud_ = std::make_shared<Program>("..\\src\\shader", "depth_point_cloud");

  point_cloud_vao_.SetDrawMode(VertexArray::DrawMode::POINTS);


  // Human label vao
  program_human_edge_ = std::make_shared<Program>("..\\src\\shader", "human_edge");

  human_label_point_vao_.BufferPointer(0, 3, human_label_buffer_, 6, 0);
  human_label_point_vao_.BufferPointer(1, 3, human_label_buffer_, 6, 3);

  human_label_edge_vao_.BufferPointer(0, 3, human_label_buffer_, 6, 0);
  human_label_edge_vao_.SetDrawElementMode(VertexArray::DrawMode::LINES, human_label_edge_elements_);
}

void Renderer::Render()
{
  UpdateCameraUniformFromScene();
  UpdateLightUniformFromScene();

  glLineWidth(2.f);

  human_label_node_ = nullptr;

  // Rendering color camera image to texture
  framebuffer_color_.Use();
  glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0, 0, 1920, 1080);

  program_color_camera_->Use();
  program_color_camera_->BindUniformBuffer(0, color_camera_uniform_);
  program_color_point_cloud_->Uniform1f("point_size", 1.f);
  program_color_point_cloud_->BindUniformBuffer(0, color_camera_uniform_);
  program_human_edge_->BindUniformBuffer(0, color_camera_uniform_);
  current_program_ = program_color_camera_;
  TraverseSceneNode(scene_->GetRootNode());

  glDisable(GL_DEPTH_TEST);
  DrawHumanLabelNodes();
  glEnable(GL_DEPTH_TEST);

  // Rendering depth camera image to texture
  framebuffer_depth_.Use();
  // The Kinect v2 can physically sense depth at 8 meters.
  // However, 4.5m is where you can reliably track body joints.
  // Anything beyond 4.5 meters your body tracking yields inconsistent results.
  unsigned int value = 80000;
  glClearBufferuiv(GL_COLOR, 0, &value);
  glClear(GL_DEPTH_BUFFER_BIT);
  glViewport(0, 0, 512, 424);

  program_depth_camera_->Use();
  current_program_ = program_depth_camera_;
  program_depth_point_cloud_->BindUniformBuffer(0, depth_camera_uniform_);
  TraverseSceneNode(scene_->GetRootNode());

  glDisable(GL_DEPTH_TEST);
  program_color_point_cloud_->BindUniformBuffer(0, depth_camera_uniform_);
  program_human_edge_->BindUniformBuffer(0, depth_camera_uniform_);
  DrawHumanLabelNodes();
  glEnable(GL_DEPTH_TEST);

  // Rendering to the screen

  Framebuffer::UseScreen();
  glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  // Full screen viewport
  //auto screen_size = engine_->GetScreenSize();
  //glViewport(0, 0, screen_size(0), screen_size(1));

  auto view_mode = engine_->GetViewMode();
  if (view_mode == Engine::ViewMode::ALL ||
    view_mode == Engine::ViewMode::VIEW)
  {
    // View camera rendering
    glViewport(0, 0, 1280, 720);
    program_color_camera_->Use();
    program_color_camera_->BindUniformBuffer(0, view_camera_uniform_);
    program_color_point_cloud_->BindUniformBuffer(0, view_camera_uniform_);
    program_color_point_cloud_->Uniform1f("point_size", 1.f);
    program_human_edge_->BindUniformBuffer(0, view_camera_uniform_);
    current_program_ = program_color_camera_;
    TraverseSceneNode(scene_->GetRootNode());

    glDisable(GL_DEPTH_TEST);
    DrawHumanLabelNodes();
  }

  glDisable(GL_DEPTH_TEST);
  switch (view_mode)
  {
  case Engine::ViewMode::ALL:
    // Color camera image from robot camera
    glViewport(0, 0, 1920 / 4, 1080 / 4);
    program_color_screen_->Use();
    framebuffer_vao_.SetTexture(0, framebuffer_color_texture_);
    framebuffer_vao_.Draw();

    // Depth camera image from robot camera
    glViewport(1920 / 4 + 10, 0, 512 / 2, 424 / 2);
    program_depth_screen_->Use();
    framebuffer_vao_.SetTexture(0, framebuffer_depth_texture_);
    framebuffer_vao_.Draw();

    // Color camera image from dataset
    glViewport(0, 1080 / 4 + 10, 1920 / 4, 1080 / 4);
    program_color_screen_->Use();
    framebuffer_vao_.SetTexture(0, textures_.find("data_color")->second);
    framebuffer_vao_.Draw();

    // Depth camera image from dataset
    glViewport(1920 / 4 + 10, 424 / 2 + 10, 512 / 2, 424 / 2);
    program_depth_screen_->Use();
    data_depth_vao_.SetTexture(0, textures_.find("data_depth")->second);
    data_depth_vao_.Draw();

    glEnable(GL_DEPTH_TEST);
    break;

  case Engine::ViewMode::COLOR_IMAGE:
    // Color camera image from dataset
    glViewport(0, 0, 1280, 720);
    program_color_screen_->Use();
    framebuffer_vao_.SetTexture(0, textures_.find("data_color")->second);
    framebuffer_vao_.Draw();
    break;

  case Engine::ViewMode::COLOR_VIEW:
    // Color camera image from robot camera
    glViewport(0, 0, 1280, 720);
    program_color_screen_->Use();
    framebuffer_vao_.SetTexture(0, framebuffer_color_texture_);
    framebuffer_vao_.Draw();
    break;

  case Engine::ViewMode::DEPTH_IMAGE:
    // Depth camera image from dataset
    glViewport(0, 0, 512, 424);
    program_depth_screen_->Use();
    data_depth_vao_.SetTexture(0, textures_.find("data_depth")->second);
    data_depth_vao_.Draw();
    break;

  case Engine::ViewMode::DEPTH_VIEW:
    // Depth camera image from robot camera
    glViewport(0, 0, 512, 424);
    program_depth_screen_->Use();
    framebuffer_vao_.SetTexture(0, framebuffer_depth_texture_);
    framebuffer_vao_.Draw();
    break;
  }

  glEnable(GL_DEPTH_TEST);
}

void Renderer::UpdateCameraUniformFromScene()
{
  auto view_camera = scene_->GetCamera("view");
  auto color_camera = scene_->GetCamera("color");
  auto depth_camera = scene_->GetCamera("depth");

  color_camera_uniform_["projection"] = color_camera->ProjectionMatrix();
  color_camera_uniform_["view"] = color_camera->ViewMatrix();
  color_camera_uniform_.Update();

  depth_camera_uniform_["projection"] = depth_camera->ProjectionMatrix();
  depth_camera_uniform_["view"] = depth_camera->ViewMatrix();
  depth_camera_uniform_.Update();

  view_camera_uniform_["projection"] = view_camera->ProjectionMatrix();
  view_camera_uniform_["view"] = view_camera->ViewMatrix();
  view_camera_uniform_.Update();

  program_depth_screen_->Uniform2f("nearfar", Vector2f(depth_camera->GetNear(), depth_camera->GetFar()));

  // TODO: width and height to screen size
  program_color_point_cloud_->Uniform4f("width_height_near_far", Vector4f(1280, 720, view_camera->GetNear(), view_camera->GetFar()));
  program_depth_point_cloud_->Uniform4f("width_height_near_far", Vector4f(512, 424, view_camera->GetNear(), view_camera->GetFar()));
}

void Renderer::UpdateLightUniformFromScene()
{
  const auto& lights = scene_->GetLights();
  for (int i = 0; i < lights.size() && i < max_num_lights_; i++)
  {
    const Light& light = lights[i];
    std::string var_name = "lights[" + std::to_string(i) + "]";

    switch (light.type)
    {
    case Light::Type::Directional:
      lights_uniform_[var_name + ".type"] = 0;
      break;

    case Light::Type::Point:
      lights_uniform_[var_name + ".type"] = 1;
      break;

    default:
      lights_uniform_[var_name + ".type"] = -1;
      break;
    }

    lights_uniform_[var_name + ".use"] = true;
    lights_uniform_[var_name + ".position"] = light.position;
    lights_uniform_[var_name + ".ambient"] = light.ambient;
    lights_uniform_[var_name + ".diffuse"] = light.diffuse;
    lights_uniform_[var_name + ".specular"] = light.specular;
    lights_uniform_[var_name + ".attenuation"] = light.attenuation;
  }
  for (int i = lights.size(); i < max_num_lights_; i++)
  {
    std::string var_name = "lights[" + std::to_string(i) + "]";
    lights_uniform_[var_name + ".use"] = false;
  }
  lights_uniform_.Update();
}

void Renderer::SetScene(std::shared_ptr<Scene> scene)
{
  scene_ = scene;
}

void Renderer::CreateEmptyTexture(const std::string& name, int width, int height, Texture::Usage usage)
{
  textures_.emplace(name, std::make_shared<Texture>(width, height, usage));
}

void Renderer::TraverseSceneNode(std::shared_ptr<SceneNode> node, Affine3d transform)
{
  if (node->IsMeshNode())
  {
    auto mesh_node = std::static_pointer_cast<MeshNode>(node);
    const auto& mesh_filename = mesh_node->GetMeshFilename();

    if (mesh_objects_.find(mesh_filename) == mesh_objects_.cend())
      mesh_objects_.emplace(mesh_filename, MeshObject(mesh_filename));

    current_program_->UniformMatrix4f("model", transform.cast<float>().matrix());
    mesh_objects_[mesh_filename].Draw();
  }

  else if (node->IsGroundNode())
  {
    current_program_->UniformMatrix4f("model", transform.cast<float>().matrix());
    //ground_vao_.Draw();
  }

  else if (node->IsPointCloudNode())
  {
    auto point_cloud_node = std::static_pointer_cast<PointCloudNode>(node);

    auto point_cloud = point_cloud_node->GetPointCloud();
    if (point_cloud != nullptr)
    {
      if (point_cloud_node->NeedUpdateBuffer())
      {
        auto n = point_cloud->NumPoints();
        point_cloud_buffer_.CopyFrom(point_cloud->GetPointBuffer(), 0, 3 * n);
        point_cloud_buffer_.CopyFrom(point_cloud->GetColorBuffer(), 3 * n, 3 * n);
        point_cloud_buffer_.Update();
        point_cloud_vao_.BufferPointer(0, 3, point_cloud_buffer_);
        point_cloud_vao_.BufferPointer(1, 3, point_cloud_buffer_, 0, 3 * n);
        point_cloud_vao_.SetVertexCount(n);

        point_cloud_node->FinishUpdateBuffer();
      }

      // TODO: switch color and depth shader
      if (current_program_ == program_color_camera_)
      {
        program_color_point_cloud_->Use();
        program_color_point_cloud_->UniformMatrix4f("model", transform.cast<float>().matrix());
      }
      else if (current_program_ == program_depth_camera_)
      {
        program_depth_point_cloud_->Use();
        program_depth_point_cloud_->UniformMatrix4f("model", transform.cast<float>().matrix());
      }

      point_cloud_vao_.Draw();

      current_program_->Use();
    }
  }
  else if (node->IsHumanLabelNode())
  {
    human_label_node_ = std::static_pointer_cast<HumanLabelNode>(node);
    human_label_ = human_label_node_->GetLabel();
    human_label_transform_ = transform;
  }

  for (auto child_node : node->GetChildren())
    TraverseSceneNode(child_node, transform * child_node->GetTransform());
}

void Renderer::DrawHumanLabelNodes()
{
  if (human_label_ == nullptr)
    return;

  // Buffer udpate
  std::vector<bool> is_valid(human_label_->NumJoints(), true);
  for (int i = 0; i < human_label_->NumJoints(); i++)
  {
    const auto& position = human_label_->Position(i);
    human_label_buffer_[i * 6 + 0] = position(0);
    human_label_buffer_[i * 6 + 1] = position(1);
    human_label_buffer_[i * 6 + 2] = position(2);

    if (position.squaredNorm() < 1e-4f)
      is_valid[i] = false;

    switch (human_label_->TrackingState(i))
    {
    case 2:
      // Tracking state 2 as red
      human_label_buffer_[i * 6 + 3] = 1.;
      human_label_buffer_[i * 6 + 4] = 0.;
      human_label_buffer_[i * 6 + 5] = 0.;
      break;

    case 1:
      // Tracking state 1 as green
      human_label_buffer_[i * 6 + 3] = 0.;
      human_label_buffer_[i * 6 + 4] = 1.;
      human_label_buffer_[i * 6 + 5] = 0.;
      break;

    case 0:
      // Unknown tracking state as black
      human_label_buffer_[i * 6 + 3] = 0.;
      human_label_buffer_[i * 6 + 4] = 0.;
      human_label_buffer_[i * 6 + 5] = 0.;
    }
  }
  human_label_buffer_.Update();
  human_label_point_vao_.SetDrawArrayMode(VertexArray::DrawMode::POINTS, human_label_->NumJoints());

  int index = 0;
  for (const auto& edge : human_label_node_->GetEdges())
  {
    if (is_valid[edge.first] && is_valid[edge.second])
    {
      human_label_edge_elements_[index++] = edge.first;
      human_label_edge_elements_[index++] = edge.second;
    }
  }
  human_label_edge_elements_.Update();
  human_label_edge_vao_.SetDrawElementMode(VertexArray::DrawMode::LINES, human_label_edge_elements_, index);

  program_human_edge_->Use();
  program_human_edge_->UniformMatrix4f("model", human_label_transform_.cast<float>().matrix());
  human_label_edge_vao_.Draw();

  program_color_point_cloud_->Use();
  program_color_point_cloud_->UniformMatrix4f("model", human_label_transform_.cast<float>().matrix());
  program_color_point_cloud_->Uniform1f("point_size", 3.f);
  human_label_point_vao_.Draw();
}
}
