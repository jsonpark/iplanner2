#ifndef IPLANNER_SCENE_POINT_CLOUD_NODE_H_
#define IPLANNER_SCENE_POINT_CLOUD_NODE_H_

#include "iplanner/scene/scene_node.h"

#include "iplanner/types.h"
#include "iplanner/data/point_cloud.h"

namespace iplanner
{
class PointCloudNode : public SceneNode
{
public:
  PointCloudNode();

  ~PointCloudNode();

  bool IsPointCloudNode() const override
  {
    return true;
  }

  void UpdateBuffer();
  bool NeedUpdateBuffer();
  void FinishUpdateBuffer();

  void SetPointCloud(std::shared_ptr<PointCloud> point_cloud);

  auto GetPointCloud() const
  {
    return point_cloud_;
  }

private:
  bool need_update_buffer_ = false;
  std::shared_ptr<PointCloud> point_cloud_;
};
}

#endif // IPLANNER_SCENE_POINT_CLOUD_NODE_H_
