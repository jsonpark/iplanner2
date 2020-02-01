#ifndef IPLANNER_DATASET_DATASET_UTKINECT_H_
#define IPLANNER_DATASET_DATASET_UTKINECT_H_

#include "iplanner/dataset/dataset.h"

#include <array>

#include "iplanner/types.h"
#include "iplanner/human/human_model.h"
#include "iplanner/human/human_label.h"

namespace iplanner
{
class UtKinect : public Dataset
{
private:
  constexpr static int num_joints_ = 20;

  using Joints = std::array<Vector3d, num_joints_>;

public:
  UtKinect() = delete;

  explicit UtKinect(const std::string& directory);

  ~UtKinect();

  int NumSequences() override;
  void SelectSequence(int idx) override;
  void SelectSequence(const std::string& name) override;
  void SelectSequenceFrame(const std::string& name, const std::string& index);
  void SelectFrame(int frame) override;

  std::string GetCurrentSequenceName() const override;

  int FrameRate() const override;
  int RgbWidth() override;
  int RgbHeight() override;
  int DepthWidth() override;
  int DepthHeight() override;

  int NumFrames() const override;
  std::vector<unsigned char> GetRgbImage() override;
  std::vector<unsigned short> GetDepthImage() override;

  std::shared_ptr<HumanModel> GetHumanModel() const override;
  std::shared_ptr<HumanLabel> GetHumanLabel() const override;

  bool PreviousSequence() override;
  bool NextSequence() override;
  bool PreviousFrame() override;
  bool NextFrame() override;

  Trajectory GetTrajectory() override;
  void SaveTrajectory(Trajectory trajectory) override;

  double CurrentTime() const override;

private:
  void LoadBody();

  void LoadTrajectory();

  std::string directory_;

  std::vector<std::string> sequence_names_;
  int current_sequence_ = -1;

  // Data info for current sequence
  int current_frame_ = -1;
  int num_frames_ = 0;
  std::vector<int> rgb_image_indices_;
  std::vector<int> depth_image_indices_;
  int cached_rgb_image_index_ = -1;
  int cached_depth_image_index_ = -1;
  std::vector<unsigned char> cached_rgb_image_;
  std::vector<unsigned short> cached_depth_image_;

  // Human model for watch-n-patch dataset
  std::shared_ptr<HumanModel> human_model_;
  std::vector<std::shared_ptr<HumanLabel>> human_labels_;

  // Data for the whole sequence
  std::vector<Joints> joints_;
  std::vector<int> joint_indices_;

  bool body_loaded_ = false;

  // Robot trajectory
  std::unique_ptr<Trajectory> trajectory_;
};
}

#endif // IPLANNER_DATASET_DATASET_UTKINECT_H_
