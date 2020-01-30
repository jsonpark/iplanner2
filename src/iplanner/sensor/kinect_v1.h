#ifndef IPLANNER_SENSOR_KINECT_V1_H_
#define IPLANNER_SENSOR_KINECT_V1_H_

#include "iplanner/types.h"
#include "iplanner/sensor/rgbd_camera.h"
#include "iplanner/data/point_cloud.h"

namespace iplanner
{
class KinectV1 : public RgbdCamera
{
private:
  static const CameraParameters color_params_;
  static const CameraParameters depth_params_;

  static const int color_width_ = 640;
  static const int color_height_ = 480;
  static const int depth_width_ = 320;
  static const int depth_height_ = 240;

public:
  KinectV1();
  ~KinectV1();

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

#endif // IPLANNER_SENSOR_KINECT_V1_H_
