#include "iplanner/dataset/dataset_occlusion.h"

#ifdef _WIN32
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

#include <iostream>
#include <fstream>
#include <algorithm>

#include <stb/stb_image.h>

#ifdef _WIN32
namespace fs = std::filesystem;
#else
namespace fs = std::experimental::filesystem;
#endif

namespace iplanner
{
namespace
{
#ifdef _WIN32
const char directory_character = '\\';
#else
const char directory_character = '/';
#endif
}

DatasetOcclusion::DatasetOcclusion(const std::string& directory)
  : directory_(directory)
{
  // Human model
  std::vector<std::string> joint_names
  {
    "head",
    "shoulder_left",
    "elbow_left",
    "wrist_left",
    "shoulder_right",
    "elbow_right",
    "wrist_right",
    "neck",
    "torso",
    "hip_left",
    "knee_left",
    "foot_left",
    "hip_right",
    "knee_right",
    "foot_right"
  };

  human_model_ = std::make_shared<HumanModel>(std::move(joint_names));


  // Get sequence names
  for (auto& it : fs::directory_iterator(directory + directory_character))
  {
    const auto& path = it.path();
    sequence_names_.push_back(path.filename().string());
  }
  std::sort(sequence_names_.begin(), sequence_names_.end());

  SelectSequence(0);
}

DatasetOcclusion::~DatasetOcclusion() = default;

int DatasetOcclusion::NumSequences()
{
  return sequence_names_.size();
}

void DatasetOcclusion::SelectSequence(int idx)
{
  if (current_sequence_ == idx)
    return;

  current_sequence_ = idx;
  current_frame_ = 0;
  cached_rgb_image_index_ = -1;
  cached_depth_image_index_ = -1;

  // Count number of images
  auto ExtractFileIndex = [](const std::string& s) {
    return std::atoi(s.substr(0, s.length() - 4).c_str());
  };

  auto index_compare = [&ExtractFileIndex](const std::string& a, const std::string& b) {
    return ExtractFileIndex(a) < ExtractFileIndex(b);
  };

  rgb_image_indices_.resize(10000);
  std::fill(rgb_image_indices_.begin(), rgb_image_indices_.end(), -1);
  for (auto& it : fs::directory_iterator(directory_ + directory_character + sequence_names_[idx] + directory_character + "rgbjpg"))
  {
    const auto& path = it.path();
    const auto& filename = path.filename().string();
    const auto index = ExtractFileIndex(filename);
    rgb_image_indices_[index] = index;
  }

  depth_image_indices_.resize(10000, -1);
  std::fill(depth_image_indices_.begin(), depth_image_indices_.end(), -1);
  for (auto& it : fs::directory_iterator(directory_ + directory_character + sequence_names_[idx] + directory_character + "depth"))
  {
    const auto& path = it.path();
    const auto& filename = path.filename().string();
    const auto index = ExtractFileIndex(filename);
    depth_image_indices_[index] = index;
  }

  // Verify rgb/depth filename matches
  int min_frame = 10000;
  int max_frame = 0;
  for (int i = 0; i < 10000; i++)
  {
    if (rgb_image_indices_[i] != -1 || depth_image_indices_[i] != -1)
    {
      if (min_frame > i)
        min_frame = i;
      if (max_frame < i)
        max_frame = i;
    }
    if (rgb_image_indices_[i] == -1 && i > 0)
      rgb_image_indices_[i] = rgb_image_indices_[i - 1];
    if (depth_image_indices_[i] == -1 && i > 0)
      depth_image_indices_[i] = depth_image_indices_[i - 1];
  }

  for (int i = max_frame - 1; i >= min_frame; i--)
  {
    if (rgb_image_indices_[i] == -1)
      rgb_image_indices_[i] = rgb_image_indices_[i + 1];
    if (depth_image_indices_[i] == -1)
      depth_image_indices_[i] = depth_image_indices_[i + 1];
  }

  for (int i = min_frame; i <= max_frame; i++)
  {
    rgb_image_indices_[i - min_frame] = rgb_image_indices_[i];
    depth_image_indices_[i - min_frame] = depth_image_indices_[i];
  }
  num_frames_ = max_frame - min_frame + 1;


  // Load body data
  LoadBody();
}

void DatasetOcclusion::LoadBody()
{
  // Loading ground truth file
  auto body_filename = directory_ + directory_character + sequence_names_[current_sequence_] + directory_character + "groundTruth.txt";

  std::ifstream in(body_filename);

  joints_.resize(num_frames_);

  double rgb_timestamp = 0.;
  double depth_timestamp = 0.;
  for (int i=0; i<num_frames_; i++)
  {
    in >> rgb_timestamp >> depth_timestamp;

    for (int j = 0; j < num_joints_; j++)
      in >> joints_[i][j](0) >> joints_[i][j](1) >> joints_[i][j](2);
  }

  // Convert joint coords to world space
  human_labels_.resize(num_frames_);
  for (int i = 0; i < num_frames_; i++)
  {
    human_labels_[i] = std::make_shared<HumanLabel>(human_model_);

    auto& human_label = *human_labels_[i];
    for (int j = 0; j < num_joints_; j++)
    {
      const auto& joint = joints_[i][j];

      // Convert milimeter to meter
      human_label.Position(j) = joint / 1000.;
    }
  }

  in.close();

  body_loaded_ = true;
}

void DatasetOcclusion::SelectSequence(const std::string& name)
{
  for (int i = 0; i < sequence_names_.size(); i++)
  {
    if (sequence_names_[i] == name)
    {
      SelectSequence(i);
      return;
    }
  }

  std::cout << "Occlusion Dataset: could not find sequence name \"" << name << "\"." << std::endl;
}

void DatasetOcclusion::SelectSequenceFrame(const std::string& name, const std::string& index)
{
  bool sequence_found = false;
  for (int i = 0; i < sequence_names_.size(); i++)
  {
    if (sequence_names_[i] == name)
    {
      SelectSequence(i);
      sequence_found = true;
      break;
    }
  }

  if (!sequence_found)
  {
    std::cout << "Occlusion Dataset: could not find sequence name \"" << name << "\"." << std::endl;
    return;
  }

  int index_number = std::atoi(index.c_str());
  for (int i = 0; i < num_frames_; i++)
  {
    if (rgb_image_indices_[i] == index_number)
    {
      SelectFrame(i);
      return;
    }
  }

  std::cout << "Occlusion Dataset: could not find frame index \"" << index << "\"." << std::endl;
}

int DatasetOcclusion::FrameRate()
{
  return 6;
}

int DatasetOcclusion::RgbWidth()
{
  return 640;
}

int DatasetOcclusion::RgbHeight()
{
  return 320;
}

int DatasetOcclusion::DepthWidth()
{
  return 640;
}

int DatasetOcclusion::DepthHeight()
{
  return 480;
}

int DatasetOcclusion::NumFrames()
{
  return num_frames_;
}

std::vector<unsigned char> DatasetOcclusion::GetRgbImage()
{
  if (cached_rgb_image_index_ == rgb_image_indices_[current_frame_])
    return cached_rgb_image_;

  cached_rgb_image_index_ = rgb_image_indices_[current_frame_];

  auto filename = directory_ + directory_character + sequence_names_[current_sequence_] + directory_character + "rgb" + directory_character + std::to_string(cached_rgb_image_index_) + ".png";
  int width, height, components;
  stbi_set_flip_vertically_on_load(true);
  unsigned char* data = stbi_load(filename.c_str(), &width, &height, &components, 0);
  cached_rgb_image_ = std::vector<unsigned char>(data, data + static_cast<std::uint64_t>(width) * height * components);
  stbi_image_free(data);

  return cached_rgb_image_;
}

std::vector<unsigned short> DatasetOcclusion::GetDepthImage()
{
  if (cached_depth_image_index_ == depth_image_indices_[current_frame_])
    return cached_depth_image_;

  cached_depth_image_index_ = depth_image_indices_[current_frame_];

  cached_depth_image_.resize(DepthWidth() * DepthHeight());

  // Depth .png images are grayscale with bit depth 16
  auto filename = directory_ + directory_character + sequence_names_[current_sequence_] + directory_character + "depth_raw" + directory_character + std::to_string(cached_depth_image_index_) + ".png";
  int width, height, components;
  stbi_set_flip_vertically_on_load(true);
  unsigned char* data = stbi_load(filename.c_str(), &width, &height, &components, 0);
  cached_depth_image_ = std::vector<unsigned short>(data, data + static_cast<std::uint64_t>(width) * height * components * sizeof(unsigned short));
  stbi_image_free(data);

  return cached_depth_image_;
}

std::shared_ptr<HumanLabel> DatasetOcclusion::GetHumanLabel() const
{
  return human_labels_[current_frame_];
}

bool DatasetOcclusion::PreviousSequence()
{
  if (current_sequence_ > 0)
  {
    SelectSequence(current_sequence_ - 1);
    return true;
  }

  return false;
}

bool DatasetOcclusion::NextSequence()
{
  if (current_sequence_ < NumSequences() - 1)
  {
    SelectSequence(current_sequence_ + 1);
    return true;
  }

  return false;
}

bool DatasetOcclusion::PreviousFrame()
{
  if (current_frame_ > 0)
  {
    current_frame_--;
    return true;
  }

  return false;
}

bool DatasetOcclusion::NextFrame()
{
  if (current_frame_ < NumFrames() - 1)
  {
    current_frame_++;
    return true;
  }

  return false;
}

void DatasetOcclusion::SelectFrame(int frame)
{
  if (frame < 0)
    current_frame_ = 0;

  else if (frame > num_frames_ - 1)
    current_frame_ = num_frames_ - 1;

  else
    current_frame_ = frame;
}
}
