#ifndef IPLANNER_DATASET_DATASET_UTKINECT_H_
#define IPLANNER_DATASET_DATASET_UTKINECT_H_

#include "iplanner/dataset/dataset.h"

namespace iplanner
{
class UtKinect : public Dataset
{
public:
  UtKinect();
  ~UtKinect();

  int NumSequences() override;
  void SelectSequence(int idx) override;
  void SelectSequence(const std::string& name) override;

  int FrameRate() override;
  int RgbWidth() override;
  int RgbHeight() override;
  int DepthWidth() override;
  int DepthHeight() override;

  int NumFrames() override;
  std::vector<unsigned char> GetRgbImage() override;
  std::vector<unsigned short> GetDepthImage() override;

  bool PreviousSequence() override;
  bool NextSequence() override;
  bool PreviousFrame() override;
  bool NextFrame() override;

private:
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
};
}

#endif // IPLANNER_DATASET_DATASET_UTKINECT_H_
