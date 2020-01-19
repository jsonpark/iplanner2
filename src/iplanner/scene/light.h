#ifndef IPLANNER_SCENE_LIGHT_H_
#define IPLANNER_SCENE_LIGHT_H_

#include <string>

#include "iplanner/types.h"

namespace iplanner
{
struct Light
{
  enum class Type : uint8_t
  {
    Directional,
    Point,
  };

  Type type = Type::Directional;
  Vector3f position{ 0.f, 0.f, 0.f };
  Vector3f ambient{ 0.f, 0.f, 0.f };
  Vector3f diffuse{ 0.f, 0.f, 0.f };
  Vector3f specular{ 0.f, 0.f, 0.f };
  Vector3f attenuation{ 0.f, 0.f, 0.f };
};
}

#endif // IPLANNER_SCENE_LIGHT_H_
