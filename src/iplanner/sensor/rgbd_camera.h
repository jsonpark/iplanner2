#ifndef IPLANNER_SENSOR_RGBD_CAMERA_H_
#define IPLANNER_SENSOR_RGBD_CAMERA_H_

#include "iplanner/types.h"
#include "iplanner/data/point_cloud.h"

namespace iplanner
{
class RgbdCamera
{
protected:
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

public:
  RgbdCamera() = default;
  ~RgbdCamera() = default;

  virtual void FeedFrame(std::vector<unsigned char>&& color, std::vector<unsigned short>&& depth) = 0;
  virtual void GeneratePointCloud() = 0;

  virtual void GetPointCloud(std::shared_ptr<PointCloud> point_cloud) const = 0;

  virtual const std::vector<unsigned char>& GetColorBuffer() const = 0;
  virtual const std::vector<unsigned short>& GetDepthBuffer() const = 0;

private:
};
}

#endif // IPLANNER_SENSOR_RGBD_CAMERA_H_
