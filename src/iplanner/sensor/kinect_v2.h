#ifndef IPLANNER_SENSOR_KINECT_V2_H_
#define IPLANNER_SENSOR_KINECT_V2_H_

#include "iplanner/types.h"
#include "iplanner/sensor/rgbd_camera.h"
#include "iplanner/data/point_cloud.h"

namespace iplanner
{
class KinectV2 : public RgbdCamera
{
private:
  static const CameraParameters color_params_;
  static const CameraParameters depth_params_;

  static const int color_width_ = 1920;
  static const int color_height_ = 1080;
  static const int depth_width_ = 512;
  static const int depth_height_ = 424;

public:
  KinectV2();
  ~KinectV2();

  void FeedFrame(std::vector<unsigned char>&& color, std::vector<unsigned short>&& depth) override;
  void GeneratePointCloud() override;

  void GetPointCloud(std::shared_ptr<PointCloud> point_cloud) const override
  {
    *point_cloud = point_cloud_;
  }

  const std::vector<unsigned char>& GetColorBuffer() const override
  {
    return color_;
  }

  const std::vector<unsigned short>& GetDepthBuffer() const override
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
