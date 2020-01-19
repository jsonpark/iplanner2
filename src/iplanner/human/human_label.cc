#include "iplanner/human/human_label.h"

namespace iplanner
{
HumanLabel::HumanLabel(std::shared_ptr<HumanModel> model)
  : model_(model),
  positions_(model->NumJoints(), Vector3d::Zero()),
  tracking_states_(model->NumJoints(), 0),
  actions_(model->NumJoints(), 0),
  is_occluded_(model->NumJoints(), 0),
  occlusion_confidences_(model->NumJoints(), 0.)
{
}

HumanLabel::~HumanLabel()
{
}
}
