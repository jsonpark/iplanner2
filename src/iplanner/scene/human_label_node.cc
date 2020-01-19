#include "iplanner/scene/human_label_node.h"

namespace iplanner
{
HumanLabelNode::HumanLabelNode() = default;

HumanLabelNode::~HumanLabelNode() = default;

void HumanLabelNode::SetLabel(std::shared_ptr<HumanLabel> label)
{
  label_ = label;
}

std::shared_ptr<HumanLabel> HumanLabelNode::GetLabel() const
{
  return label_;
}
}
