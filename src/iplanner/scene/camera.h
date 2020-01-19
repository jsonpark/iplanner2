#ifndef IPLANNER_SCENE_CAMERA_H_
#define IPLANNER_SCENE_CAMERA_H_

#include <string>

#include "iplanner/types.h"

namespace iplanner
{
class Camera
{
private:
  static constexpr double PI = 3.1415926535897932384626433832795;
  static constexpr float PI_F = (float)PI;

  enum class ProjectionType
  {
    ORTHOGONAL,
    PERSPECTIVE,
  };

public:
  Camera();
  ~Camera();

  void Load(const std::string& filename);
  void Save(const std::string& filename);

  void SetPerspective() noexcept
  {
    projection_type_ = ProjectionType::PERSPECTIVE;
  }

  void SetOrthogonal() noexcept
  {
    projection_type_ = ProjectionType::ORTHOGONAL;
  }

  void SetAspect(float aspect) noexcept
  {
    aspect_ = aspect;
  }

  void SetFovy(float fovy) noexcept
  {
    fovy_ = fovy;
  }

  void SetNear(float nearf) noexcept
  {
    near_ = nearf;
  }

  auto GetNear() const noexcept
  {
    return near_;
  }

  void SetFar(float farf) noexcept
  {
    far_ = farf;
  }

  auto GetFar() const noexcept
  {
    return far_;
  }

  void SetEye(float x, float y, float z)
  {
    eye_ = Vector3f(x, y, z);
  }

  void SetEye(const Vector3f& eye)
  {
    eye_ = eye;
  }

  void SetCenter(float x, float y, float z)
  {
    center_ = Vector3f(x, y, z);
  }

  void SetCenter(const Vector3f& center)
  {
    center_ = center;
  }

  void SetUp(float x, float y, float z)
  {
    up_ = Vector3f(x, y, z);
  }

  void SetUp(const Vector3f& up)
  {
    up_ = up;
  }

  const auto& GetEye() const noexcept
  {
    return eye_;
  }

  void SetWidthHeight(int width, int height)
  {
    width_ = width;
    height_ = height;
  }

  void SetTransform(const Affine3d& transform);

  Matrix4f ProjectionMatrix();
  Matrix4f ViewMatrix();

  void Translate(float x, float y);
  void Zoom(float z);
  void Rotate(float x, float y);

private:
  ProjectionType projection_type_ = ProjectionType::PERSPECTIVE;

  Vector3f eye_{ 0.f, -1.f, 0.f };
  Vector3f up_{ 0.f, 0.f, 1.f };
  Vector3f center_{ 0.f, 0.f, 0.f };

  float aspect_ = 800.f / 600.f;
  float fovy_ = 60.f / (PI_F / 180.f);
  float near_ = 0.1f;
  float far_ = 100.f;

  float width_ = 800.f;
  float height_ = 600.f;

  float translate_sensitivity_ = 0.01f;
  float zoom_sensitivity_ = 0.01f;
  float rotate_sensitivity_ = 0.003f;
};
}

#endif // IPLANNER_SCENE_CAMERA_H_
