#include "iplanner/sensor/kinect_v2.h"

#include <iostream>

namespace iplanner
{
const KinectV2::CameraParameters KinectV2::color_params_ =
{
  // Intrinsic
  .fx = 1050.60702,
  .fy = 1052.77088,
  .cx = 983.72076,
  .cy = 530.46047,

  // Distortion
  .k1 = 0.02468,
  .k2 = -0.02250,
  .p1 = -0.00338,
  .p2 = 0.00482,
  .k3 = 0.00000,
};

const KinectV2::CameraParameters KinectV2::depth_params_ =
{
  // Intrinsic
  .fx = 365.19599,
  .fy = 365.61839,
  .cx = 266.84507,
  .cy = 199.14226,

  // Distortion
  .k1 = 0.11926,
  .k2 = -0.53343,
  .p1 = -0.00021,
  .p2 = 0.00730,
  .k3 = 0.00000,
};

KinectV2::KinectV2()
{
  rotation_ << 0.999971346110444, -0.00756469826720314, 0.000286876615262462,
    0.00756522677983534, 0.999969600410407, -0.00188827932806725,
    -0.000272583630970059, 0.00189039550817223, 0.999998176049830;

  translation_ << -56.7170118956704, -0.728510987903765, -3.87867708001932;
}

KinectV2::~KinectV2() = default;

void KinectV2::FeedFrame(std::vector<unsigned char>&& color, std::vector<unsigned short>&& depth)
{
  color_ = std::move(color);
  depth_ = std::move(depth);
}

void KinectV2::GeneratePointCloud()
{
  std::vector<float> coords;
  std::vector<float> colors;

  coords.resize(depth_width_ * depth_height_ * 3);

  // Depth plane to depth world
  for (int x = 1; x <= depth_width_; x++)
  {
    for (int y = 1; y <= depth_height_; y++)
    {
      //int index = (y - 1) + (x - 1) * depth_height_;
      int index = (x - 1) + (depth_height_ - (y - 1) - 1) * depth_width_;
      coords[index * 3 + 0] = (x - depth_params_.cx) * depth_[index] / depth_params_.fx;
      coords[index * 3 + 1] = (y - depth_params_.cy) * depth_[index] / depth_params_.fy;
      coords[index * 3 + 2] = depth_[index];
    }
  }

  // Depth world to color world to color plane
  colors.resize(depth_width_ * depth_height_ * 3);
  std::fill(colors.begin(), colors.end(), 0.f);
  for (int i = 0; i < colors.size() / 3; i++)
  {
    Vector3f color_plane = (rotation_ * Vector3d(coords[i * 3 + 0], coords[i * 3 + 1], coords[i * 3 + 2]) + translation_).cast<float>();
    color_plane(0) = color_plane(0) * color_params_.fx / color_plane(2) + color_params_.cx;
    color_plane(1) = color_plane(1) * color_params_.fy / color_plane(2) + color_params_.cy;

    Vector3f rounded(std::roundf(color_plane(0)), std::roundf(color_plane(1)), std::roundf(color_plane(2)));
    if (0 < rounded(0) && rounded(0) <= color_width_ &&
      0 < rounded(1) && rounded(1) <= color_height_ &&
      rounded(2) != 0.f)
    {
      int depth_index = i;
      int color_index = (rounded(0) - 1) + (color_height_ - (rounded(1) - 1) - 1) * color_width_;

      colors[depth_index * 3 + 0] = color_[color_index * 3 + 0] / 255.f;
      colors[depth_index * 3 + 1] = color_[color_index * 3 + 1] / 255.f;
      colors[depth_index * 3 + 2] = color_[color_index * 3 + 2] / 255.f;
    }
  }

  // Convert millimeter unit to meter
  for (int i = 0; i < coords.size(); i++)
    coords[i] /= 1000.f;

  point_cloud_.SetBuffers(std::move(coords), std::move(colors));
}
}
