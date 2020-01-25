#include "iplanner/sensor/fake_kinect.h"

#include <iostream>

namespace iplanner
{
/*
#define ROBONECT_CAMERA_WIDTH   640
#define ROBONECT_CAMERA_HEIGHT  480

#define ROBONECT_CAMERA_FX      521.179233
#define ROBONECT_CAMERA_FY      493.033034

#define ROBONECT_CAMERA_CX      322.515987
#define ROBONECT_CAMERA_CY      259.055966



#define K1  5.858325e-02
#define K2  3.856792e-02
#define P1  0
#define P2  0
#define K3  0
*/
const FakeKinect::CameraParameters FakeKinect::camera_params_ =
{
  // Intrinsic
  .fx = 521.179233,
  .fy = 493.033034,
  .cx = 322.515987,
  .cy = 259.055966,

  // Distortion
  .k1 = 5.858325e-02,
  .k2 = 3.856792e-02,
  .p1 = 0.,
  .p2 = 0.,
  .k3 = 0.,
};

FakeKinect::FakeKinect()
{
  rotation_ = Matrix3d::Identity();
  translation_ = Vector3d::Zero();
}

FakeKinect::~FakeKinect() = default;

void FakeKinect::FeedFrame(std::vector<unsigned char>&& color, std::vector<unsigned short>&& depth)
{
  color_ = std::move(color);
  depth_ = std::move(depth);
}

void FakeKinect::GeneratePointCloud()
{
  std::vector<float> coords;
  std::vector<float> colors;

  coords.resize(width_ * height_ * 3);
  colors.resize(width_ * height_ * 3);
  std::fill(colors.begin(), colors.end(), 0.f);

  // Depth plane to depth world
  for (int x = 1; x <= width_; x++)
  {
    for (int y = 1; y <= height_; y++)
    {
      int index = (x - 1) + (height_ - (y - 1) - 1) * width_;
      coords[index * 3 + 0] = (x - camera_params_.cx) * depth_[index] / camera_params_.fx;
      coords[index * 3 + 1] = (y - camera_params_.cy) * depth_[index] / camera_params_.fy;
      coords[index * 3 + 2] = depth_[index];
    }
  }
  /*
  for (int x = 1; x <= width_; x++)
  {
    for (int y = 1; y <= height_; y++)
    {
      int index = (y - 1) + (x - 1) * height_;
      coords[index * 3 + 0] = (x - camera_params_.cx) * depth_[index] / camera_params_.fx;
      coords[index * 3 + 1] = (y - camera_params_.cy) * depth_[index] / camera_params_.fy;
      coords[index * 3 + 2] = depth_[index];
    }
  }
  */

  // Depth world to color world to color plane
  for (int i = 0; i < colors.size() / 3; i++)
  {
    colors[i * 3 + 0] = color_[i * 3 + 0] / 255.f;
    colors[i * 3 + 1] = color_[i * 3 + 1] / 255.f;
    colors[i * 3 + 2] = color_[i * 3 + 2] / 255.f;
  }

  /*
  for (int i = 0; i < colors.size() / 3; i++)
  {
    Vector3f color_plane = (rotation_ * Vector3d(coords[i * 3 + 0], coords[i * 3 + 1], coords[i * 3 + 2]) + translation_).cast<float>();
    color_plane(0) = color_plane(0) * camera_params_.fx / color_plane(2) + camera_params_.cx;
    color_plane(1) = color_plane(1) * camera_params_.fy / color_plane(2) + camera_params_.cy;

    Vector3f rounded(std::roundf(color_plane(0)), std::roundf(color_plane(1)), std::roundf(color_plane(2)));
    if (0 < rounded(0) && rounded(0) <= width_ &&
      0 < rounded(1) && rounded(1) <= height_ &&
      rounded(2) != 0.f)
    {
      int depth_index = i;
      int color_index = (rounded(0) - 1) + (height_ - (rounded(1) - 1) - 1) * width_;

      colors[depth_index * 3 + 0] = color_[color_index * 3 + 0] / 255.f;
      colors[depth_index * 3 + 1] = color_[color_index * 3 + 1] / 255.f;
      colors[depth_index * 3 + 2] = color_[color_index * 3 + 2] / 255.f;
    }
  }
  */

  // Convert millimeter unit to meter
  for (int i = 0; i < coords.size(); i++)
    coords[i] /= 1000.f;

  point_cloud_.SetBuffers(std::move(coords), std::move(colors));
}
}
