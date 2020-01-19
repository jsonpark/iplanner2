#ifndef IPLANNER_HUMAN_HUMAN_LABEL_H_
#define IPLANNER_HUMAN_HUMAN_LABEL_H_

#include <vector>
#include <string>

#include "iplanner/human/human_model.h"
#include "iplanner/types.h"

namespace iplanner
{
class HumanLabel
{
  enum class Action : uint8_t
  {
    UNKNOWN,
  };

public:
  HumanLabel() = delete;

  explicit HumanLabel(std::shared_ptr<HumanModel> model);

  ~HumanLabel();

  auto NumJoints() const
  {
    return model_->NumJoints();
  }

  auto& Position(int i)
  {
    return positions_[i];
  }

  const auto& Position(int i) const
  {
    return positions_[i];
  }

  auto& TrackingState(int i)
  {
    return tracking_states_[i];
  }

  auto TrackingState(int i) const
  {
    return tracking_states_[i];
  }

  auto& Action(int i)
  {
    return actions_[i];
  }

  auto Action(int i) const
  {
    return actions_[i];
  }

  auto& IsOccluded(int i)
  {
    return is_occluded_[i];
  }

  auto IsOccluded(int i) const
  {
    return is_occluded_[i];
  }

  auto& OcclusionConfidence(int i)
  {
    return occlusion_confidences_[i];
  }

  auto OcclusionConfidence(int i) const
  {
    return occlusion_confidences_[i];
  }

private:
  std::shared_ptr<HumanModel> model_;

  // From watch-n-patch dataset
  Vector3dVector positions_;
  std::vector<int> tracking_states_;
  std::vector<int> actions_;

  // Other labels
  std::vector<int> is_occluded_;
  std::vector<double> occlusion_confidences_;
};
}

#endif // IPLANNER_HUMAN_HUMAN_LABEL_H_
