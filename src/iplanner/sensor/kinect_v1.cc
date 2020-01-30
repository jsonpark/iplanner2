#include "iplanner/sensor/kinect_v1.h"

#include <iostream>

namespace iplanner
{
/*
const KinectV1::CameraParameters KinectV1::color_params_ =
{
  // Intrinsic
  .fx = 516.27251,
  .fy = 516.52227,
  .cx = 349.21872, // .cx = 319.21872,
  .cy = 265.29767, // .cy = 225.29767,

  // Distortion
  .k1 = 0.108983708314500,
  .k2 = -0.239830795000911,
  .p1 = -0.001984065259398,
  .p2 = -0.002884597433015,
  .k3 = 0.,
};

const KinectV1::CameraParameters KinectV1::depth_params_ =
{
  // Intrinsic
  .fx = 591.81690,
  .fy = 591.23953,
  .cx = 316.77227,
  .cy = 249.56119,

  // Distortion
  .k1 = -0.127613248346941,
  .k2 = 0.470606007302241,
  .p1 = 0.000048478690145,
  .p2 = 0.017448057052172,
  .k3 = 0.000000000000000,
};
*/

const KinectV1::CameraParameters KinectV1::color_params_ =
{
  // Intrinsic
  .fx = 521.179233,
  .fy = 493.033034,
  .cx = 322.515987, // .cx = 319.21872,
  .cy = 259.055966, // .cy = 225.29767,

  // Distortion
  .k1 = 0.108983708314500,
  .k2 = -0.239830795000911,
  .p1 = -0.001984065259398,
  .p2 = -0.002884597433015,
  .k3 = 0.,
};

const KinectV1::CameraParameters KinectV1::depth_params_ =
{
  // Intrinsic
  .fx = 521.179233,
  .fy = 493.033034,
  .cx = 322.515987,
  .cy = 259.055966,

  // Distortion
  .k1 = -0.127613248346941,
  .k2 = 0.470606007302241,
  .p1 = 0.000048478690145,
  .p2 = 0.017448057052172,
  .k3 = 0.000000000000000,
};

KinectV1::KinectV1()
{
  rotation_ << 1.0000, -0.0009, 0.0006,
    0.0009, 0.9999, 0.0134,
    -0.0006, -0.0134, 0.9999;
  translation_ << -25.36961, -1.15194, -7.98245;
}

KinectV1::~KinectV1() = default;

void KinectV1::FeedFrame(std::vector<unsigned char>&& color, std::vector<unsigned short>&& depth)
{
  color_ = std::move(color);
  depth_ = std::move(depth);
}

void KinectV1::GeneratePointCloud()
{
  std::vector<float> coords;
  std::vector<float> colors;

  coords.resize(depth_width_ * depth_height_ * 3);
  colors.resize(depth_width_ * depth_height_ * 3);
  std::fill(colors.begin(), colors.end(), 0.f);

  // Depth plane to depth world
  for (int x = 1; x <= depth_width_; x++)
  {
    for (int y = 1; y <= depth_height_; y++)
    {
      int index = (x - 1) + (depth_height_ - (y - 1) - 1) * depth_width_;
      coords[index * 3 + 0] = (2 * x - depth_params_.cx) * depth_[index] / depth_params_.fx;
      coords[index * 3 + 1] = (2 * y - depth_params_.cy) * depth_[index] / depth_params_.fy;
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
  /*
  for (int x = 1; x <= depth_width_; x++)
  {
    for (int y = 1; y <= depth_height_; y++)
    {
      //int index = (x - 1) + (depth_height_ - (y - 1) - 1) * depth_width_;
      int coord_index = (x - 1) + (y - 1) * depth_width_;
      int color_index = 2 * (x - 1) + 4 * (depth_height_ - (y - 1) - 1) * depth_width_;
      colors[coord_index * 3 + 0] = color_[color_index * 3 + 0] / 255.f;
      colors[coord_index * 3 + 1] = color_[color_index * 3 + 1] / 255.f;
      colors[coord_index * 3 + 2] = color_[color_index * 3 + 2] / 255.f;
    }
  }
  */

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
