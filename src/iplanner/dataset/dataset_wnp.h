#ifndef IPLANNER_DATASET_DATASET_WNP_H_
#define IPLANNER_DATASET_DATASET_WNP_H_

#include "iplanner/dataset/dataset.h"

#include <type_traits>
#include <array>

/*
#include <MatlabEngine.hpp>
#include <MatlabExecutionInterface.hpp>
*/

#include "iplanner/types.h"
#include "iplanner/human/human_model.h"
#include "iplanner/human/human_label.h"

namespace iplanner
{
class Wnp : public Dataset
{
private:
  static const std::vector<std::string> scene_names_;

public:
  struct Joint
  {
    int tracking_state;
    Vector3d camera;
    Vector2d color;
    Vector2d depth;
    Quaterniond rotation;
    Vector3d pcloud;
  };

  using Joints = std::array<Joint, 25>;

public:
  Wnp() = delete;

  explicit Wnp(const std::string& directory);

  ~Wnp();
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
  void LoadBodyFromMatFile();
  void LoadBodyFromStructFile();

  void LoadTrajectory();

  const std::string directory_;

  std::vector<std::string> sequence_names_;
  int current_sequence_ = -1;

  // Data for current sequence
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
  std::vector<bool> is_body_tracked_;

  // Matlab engine
  /*
  matlab::execution::FutureResult<std::unique_ptr<matlab::engine::MATLABEngine>> matlab_future_;
  std::unique_ptr<matlab::engine::MATLABEngine> matlab_;
  */

  bool body_loaded_ = false;

  // Robot trajectory
  std::unique_ptr<Trajectory> trajectory_;
};
}

#endif // IPLANNER_DATASET_DATASET_WNP_H_
