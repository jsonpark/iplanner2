#ifndef IPLANNER_SENSOR_FAKE_KINECT_H_
#define IPLANNER_SENSOR_FAKE_KINECT_H_

#include "iplanner/types.h"
#include "iplanner/sensor/rgbd_camera.h"
#include "iplanner/data/point_cloud.h"

namespace iplanner
{
class FakeKinect : public RgbdCamera
{
private:
  static const CameraParameters camera_params_;

  static const int width_ = 640;
  static const int height_ = 480;

public:
  FakeKinect();
  ~FakeKinect();

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

#endif // IPLANNER_SENSOR_FAKE_KINECT_H_
