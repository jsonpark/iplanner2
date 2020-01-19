#include "iplanner/human/human_model.h"

namespace iplanner
{
HumanModel::HumanModel(const std::vector<std::string>& joint_names)
  : joint_names_(joint_names)
{
}

HumanModel::HumanModel(std::vector<std::string>&& joint_names)
  : joint_names_(std::move(joint_names))
{
}

HumanModel::~HumanModel()
{
}
}
