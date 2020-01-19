#ifndef IPLANNER_HUMAN_HUMAN_MODEL_H_
#define IPLANNER_HUMAN_HUMAN_MODEL_H_

#include <vector>
#include <string>

#include "iplanner/types.h"

namespace iplanner
{
class HumanModel
{
public:
  HumanModel() = delete;

  explicit HumanModel(const std::vector<std::string>& joint_names);

  explicit HumanModel(std::vector<std::string>&& joint_names);

  ~HumanModel();

  int NumJoints() const noexcept
  {
    return joint_names_.size();
  }

  const auto& JointName(int i) const
  {
    return joint_names_[i];
  }

private:
  std::vector<std::string> joint_names_;
};
}

#endif // IPLANNER_HUMAN_HUMAN_MODEL_H_
