#include "iplanner/scene/scene.h"

namespace iplanner
{
Scene::Scene()
{
  root_ = std::make_shared<SceneNode>();
}

Scene::~Scene()
{
}

std::shared_ptr<Camera> Scene::GetCamera(const std::string& name) const
{
  auto it = cameras_.find(name);
  if (it == cameras_.cend())
    return nullptr;
  return it->second;
}

void Scene::SetCamera(const std::string& name, std::shared_ptr<Camera> camera)
{
  cameras_[name] = camera;
}

void Scene::AddLight(const Light& light)
{
  lights_.push_back(light);
}
}
