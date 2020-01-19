#ifndef IPLANNER_SCENE_SCENE_H_
#define IPLANNER_SCENE_SCENE_H_

#include <memory>

#include "iplanner/scene/camera.h"
#include "iplanner/scene/scene_node.h"
#include "iplanner/scene/light.h"

namespace iplanner
{
class Scene
{
public:
  Scene();
  ~Scene();

  void SetCamera(const std::string& name, std::shared_ptr<Camera> camera);

  void AddLight(const Light& light);

  std::shared_ptr<Camera> GetCamera(const std::string& name) const;

  auto GetRootNode() const
  {
    return root_;
  }

  const auto& GetLights() const
  {
    return lights_;
  }

private:
  std::unordered_map<std::string, std::shared_ptr<Camera>> cameras_;

  std::shared_ptr<SceneNode> root_;

  std::vector<Light> lights_;
};
}

#endif // IPLANNER_SCENE_SCENE_H_
