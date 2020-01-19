#include "iplanner/scene/camera.h"

#include <fstream>

namespace iplanner
{
Camera::Camera()
{
}

Camera::~Camera()
{
}

void Camera::Load(const std::string& filename)
{
  std::ifstream file(filename);
  if (!file.is_open())
    return;

  std::string type;
  file >> type;
  if (type == "PERSPECTIVE")
    projection_type_ = ProjectionType::PERSPECTIVE;
  else
    projection_type_ = ProjectionType::ORTHOGONAL;

  file >> eye_(0) >> eye_(1) >> eye_(2)
    >> up_(0) >> up_(1) >> up_(2)
    >> center_(0) >> center_(1) >> center_(2)
    >> aspect_ >> fovy_ >> near_ >> far_
    >> width_ >> height_;

  file.close();
}

void Camera::Save(const std::string& filename)
{
  std::ofstream file(filename);
  if (!file.is_open())
    return;

  switch (projection_type_)
  {
  case ProjectionType::PERSPECTIVE:
    file << "PERSPECTIVE\n";
    break;

  case ProjectionType::ORTHOGONAL:
    file << "ORTHOGONAL\n";
    break;
  }

  file << eye_.transpose() << "\n"
    << up_.transpose() << "\n"
    << center_.transpose() << "\n"
    << aspect_ << "\n"
    << fovy_ << "\n"
    << near_ << "\n"
    << far_ << "\n"
    << width_ << "\n"
    << height_ << "\n";

  file.close();
}

void Camera::SetTransform(const Affine3d& transform)
{
  Affine3f f_transform = transform.cast<float>();
  eye_ = f_transform.translation();
  center_ = eye_ + f_transform.matrix().block(0, 1, 3, 1);
  up_ = f_transform.matrix().block(0, 2, 3, 1);
}

Matrix4f Camera::ProjectionMatrix()
{
  Matrix4f p;
  p.setIdentity();

  switch (projection_type_)
  {
  case ProjectionType::ORTHOGONAL:
    p(0, 0) = 2.f / width_;
    p(1, 1) = 2.f / height_;
    p(2, 2) = -2.f / (far_ - near_);
    p(2, 3) = -(far_ + near_) / (far_ - near_);
    break;

  case ProjectionType::PERSPECTIVE:
    p(1, 1) = 1.f / tanf(fovy_ / 2.f);
    p(0, 0) = p(1, 1) / aspect_;
    p(2, 2) = -(far_ + near_) / (far_ - near_);
    p(2, 3) = -2.f * far_ * near_ / (far_ - near_);
    p(3, 2) = -1.f;
    p(3, 3) = 0.f;
    break;
  }

  return p;
}

Matrix4f Camera::ViewMatrix()
{
  Vector3f z = (eye_ - center_).normalized();
  Vector3f x = up_.cross(z).normalized();
  Vector3f y = z.cross(x);

  Matrix4f view;
  view.setIdentity();
  view.block(0, 0, 1, 3) = x.transpose();
  view.block(1, 0, 1, 3) = y.transpose();
  view.block(2, 0, 1, 3) = z.transpose();

  Matrix4f transpose;
  transpose.setIdentity();
  transpose.block(0, 3, 3, 1) = -eye_;

  return view * transpose;
}

void Camera::Translate(float x, float y)
{
  x *= -translate_sensitivity_;
  y *= translate_sensitivity_;

  Matrix3f m;
  m.col(1) = up_.cross(eye_ - center_).normalized();
  m.col(2) = (eye_ - center_).cross(m.col(1)).normalized();

  eye_ += m.col(1) * x + m.col(2) * y;
  center_ += m.col(1) * x + m.col(2) * y;
}

void Camera::Zoom(float z)
{
  z *= zoom_sensitivity_;

  constexpr float min_distance = 0.01f;

  float d = (eye_ - center_).norm();
  if (z + d < min_distance)
    d = min_distance - z;

  eye_ = center_ + (z + d) * (eye_ - center_).normalized();
}

void Camera::Rotate(float x, float y)
{
  x *= -rotate_sensitivity_;
  y *= -rotate_sensitivity_;

  constexpr float min_angle = 0.01f;

  Vector3f v = eye_ - center_;

  Matrix3f m;
  m.col(1) = up_.cross(v).normalized();
  m.col(0) = m.col(1).cross(up_).normalized();
  m.col(2) = m.col(0).cross(m.col(1));

  float current_yangle = acosf(up_.dot(v) / v.norm());
  if (y < 0 && current_yangle + y < min_angle)
    y = min_angle - current_yangle;
  else if (y > 0 && current_yangle + y > PI - min_angle)
    y = PI_F - min_angle - current_yangle;

  Matrix3f r1 = AngleAxisf(y, Vector3f(0.f, 1.f, 0.f)).matrix();
  Matrix3f r2 = AngleAxisf(x, Vector3f(0.f, 0.f, 1.f)).matrix();

  eye_ = center_ + m * r2 * r1 * m.transpose() * v;
}
}
