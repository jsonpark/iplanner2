#include "iplanner/dataset/dataset_utkinect.h"

#ifdef _WIN32
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

#include <iostream>
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

// Extract file index XXXX from filename format "colorImgXXXX.jpg" and "depthImgXXXX.xml"
int ExtractFileIndex(const std::string& filename)
{
  int len = filename.length();

  // Minimum length is 8 (base) + 1 (index) + 4 (extension) = 13
  // Maximum length is 16
  if (13 <= len && len <= 16 &&
    (filename.substr(0, 8) == "colorImg" && filename.substr(len - 4) == ".jpg" ||
    filename.substr(0, 8) == "depthImg" && filename.substr(len - 4) == ".xml"))
    return std::stoi(filename.substr(8, len - 12));

  return 10000;
}
}

UtKinect::UtKinect(const std::string& directory)
  : directory_(directory)
{
  // Get sequence names
  for (auto& it : fs::directory_iterator(directory + "RGB" + directory_character))
  {
    const auto& path = it.path();
    sequence_names_.push_back(path.filename().string());
  }
  std::sort(sequence_names_.begin(), sequence_names_.end());

  // Verify sequence names from depth data
  std::vector<std::string> depth_sequence_names;
  for (auto& it : fs::directory_iterator(directory + "depth" + directory_character))
  {
    const auto& path = it.path();
    depth_sequence_names.push_back(path.filename().string());
  }
  std::sort(depth_sequence_names.begin(), depth_sequence_names.end());

  if (sequence_names_.size() != depth_sequence_names.size())
    throw std::runtime_error("UtKinect: sequence numbers of RGB and depth mismatch");

  for (int i = 0; i < sequence_names_.size(); i++)
  {
    if (sequence_names_[i] != depth_sequence_names[i])
      throw std::runtime_error("UtKinect: sequence names of RGB and depth mismatch:\n"
        + std::to_string(i) + "-th sequence names: \"" + sequence_names_[i] + "\" and \"" + depth_sequence_names[i] + "\"");
  }

  SelectSequence(0);
}

UtKinect::~UtKinect()
{
}

int UtKinect::NumSequences()
{
  return sequence_names_.size();
}

void UtKinect::SelectSequence(int idx)
{
  if (current_sequence_ == idx)
    return;

  current_sequence_ = idx;
  current_frame_ = 0;
  cached_rgb_image_index_ = -1;
  cached_depth_image_index_ = -1;

  // Count number of images
  auto index_compare = [](const std::string& a, const std::string& b) {
    return ExtractFileIndex(a) < ExtractFileIndex(b);
  };

  rgb_image_indices_.resize(10000);
  std::fill(rgb_image_indices_.begin(), rgb_image_indices_.end(), -1);
  for (auto& it : fs::directory_iterator(directory_ + directory_character + "RGB" + directory_character + sequence_names_[idx] + directory_character))
  {
    const auto& path = it.path();
    const auto& filename = path.filename().string();
    const auto index = ExtractFileIndex(filename);
    rgb_image_indices_[index] = index;
  }

  depth_image_indices_.resize(10000, -1);
  std::fill(depth_image_indices_.begin(), depth_image_indices_.end(), -1);
  for (auto& it : fs::directory_iterator(directory_ + directory_character + "depth" + directory_character + sequence_names_[idx] + directory_character))
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
}

void UtKinect::SelectSequence(const std::string& name)
{
  for (int i = 0; i < sequence_names_.size(); i++)
  {
    if (sequence_names_[i] == name)
    {
      SelectSequence(i);
      return;
    }
  }

  std::cout << "UtKinect: could not find sequence name \"" << name << "\"." << std::endl;
}

int UtKinect::FrameRate()
{
  return 60;
}

int UtKinect::RgbWidth()
{
  return 640;
}

int UtKinect::RgbHeight()
{
  return 480;
}

int UtKinect::DepthWidth()
{
  return 320;
}

int UtKinect::DepthHeight()
{
  return 240;
}

int UtKinect::NumFrames()
{
  return num_frames_;
}

std::vector<unsigned char> UtKinect::GetRgbImage()
{
  if (cached_rgb_image_index_ == rgb_image_indices_[current_frame_])
    return cached_rgb_image_;

  cached_rgb_image_index_ = rgb_image_indices_[current_frame_];

  auto filename = directory_ + directory_character + "RGB" + directory_character + sequence_names_[current_sequence_] + directory_character + "colorImg" + std::to_string(cached_rgb_image_index_) + ".jpg";
  int width, height, components;
  stbi_set_flip_vertically_on_load(true);
  unsigned char* data = stbi_load(filename.c_str(), &width, &height, &components, 0);
  cached_rgb_image_ = std::vector<unsigned char>(data, data + static_cast<std::uint64_t>(width) * height * components);
  stbi_image_free(data);

  return cached_rgb_image_;
}

std::vector<unsigned short> UtKinect::GetDepthImage()
{
  if (cached_depth_image_index_ == depth_image_indices_[current_frame_])
    return cached_depth_image_;

  cached_depth_image_index_ = depth_image_indices_[current_frame_];

  // TODO
  auto filename = directory_ + directory_character + "depth" + directory_character + sequence_names_[current_sequence_] + directory_character + "depthImg" + std::to_string(cached_depth_image_index_) + ".xml";

  return cached_depth_image_;
}

bool UtKinect::PreviousSequence()
{
  if (current_sequence_ > 0)
  {
    SelectSequence(current_sequence_ - 1);
    return true;
  }

  return false;
}

bool UtKinect::NextSequence()
{
  if (current_sequence_ < NumSequences() - 1)
  {
    SelectSequence(current_sequence_ + 1);
    return true;
  }

  return false;
}

bool UtKinect::PreviousFrame()
{
  if (current_frame_ > 0)
  {
    current_frame_--;
    return true;
  }

  return false;
}

bool UtKinect::NextFrame()
{
  if (current_frame_ < NumFrames() - 1)
  {
    current_frame_++;
    return true;
  }

  return false;
}
}
