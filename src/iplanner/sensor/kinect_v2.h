#ifndef IPLANNER_SENSOR_KINECT_V2_H_
#define IPLANNER_SENSOR_KINECT_V2_H_

#include "iplanner/types.h"
#include "iplanner/data/point_cloud.h"

namespace iplanner
{
class KinectV2
{
private:
  struct CameraParameters
  {
    // Intrinsic
    double fx;
    double fy;
    double cx;
    double cy;

    // Distortion
    double k1;
    double k2;
    double p1;
    double p2;
    double k3;
  };

  static const CameraParameters color_params_;
  static const CameraParameters depth_params_;

  static const int color_width_ = 1920;
  static const int color_height_ = 1080;
  static const int depth_width_ = 512;
  static const int depth_height_ = 424;

public:
  KinectV2();
  ~KinectV2();

  void FeedFrame(std::vector<unsigned char>&& color, std::vector<unsigned short>&& depth);
  void GeneratePointCloud();

  void GetPointCloud(std::shared_ptr<PointCloud> point_cloud) const
  {
    *point_cloud = point_cloud_;
  }

  const auto& GetColorBuffer() const
  {
    return color_;
  }

  const auto& GetDepthBuffer() const
  {
    return depth_;
  }

private:
  Matrix3d rotation_;
  Vector3d translation_;

  std::vector<unsigned char> color_;
  std::vector<unsigned short> depth_;

  PointCloud point_cloud_;
};
}

#endif // IPLANNER_SENSOR_KINECT_V2_H_
