#include "iplanner/scene/point_cloud_node.h"

namespace iplanner
{
PointCloudNode::PointCloudNode() = default;

PointCloudNode::~PointCloudNode() = default;

void PointCloudNode::SetPointCloud(std::shared_ptr<PointCloud> point_cloud)
{
  point_cloud_ = point_cloud;
  need_update_buffer_ = true;
}

void PointCloudNode::UpdateBuffer()
{
  need_update_buffer_ = true;
}

bool PointCloudNode::NeedUpdateBuffer()
{
  return need_update_buffer_;
}

void PointCloudNode::FinishUpdateBuffer()
{
  need_update_buffer_ = false;
}
}
