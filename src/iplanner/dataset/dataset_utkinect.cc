#include "iplanner/dataset/dataset_utkinect.h"

#ifdef _WIN32
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

#include <iostream>
#include <algorithm>
#include <fstream>

#include <stb/stb_image.h>
#include <tinyxml2/tinyxml2.h>

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
  // Human model
  std::vector<std::string> joint_names
  {
    "Hip",
    "Spine",
    "ShouldersCenter",
    "Head",
    "LeftShoulder",
    "LeftElbow",
    "LeftWrist",
    "LeftHand",
    "RightShoulder",
    "RightElbow",
    "RightWrist",
    "RightHand",
    "LeftHip",
    "LeftKnee",
    "LeftAnkle",
    "LeftFoot",
    "RightHip",
    "RightKnee",
    "RightAnkle",
    "RightFoot",
  };

  human_model_ = std::make_shared<HumanModel>(std::move(joint_names));


  // Get sequence names
  for (auto& it : fs::directory_iterator(directory + directory_character + "RGB" + directory_character))
  {
    const auto& path = it.path();
    sequence_names_.push_back(path.filename().string());
  }
  std::sort(sequence_names_.begin(), sequence_names_.end());

  // Verify sequence names from depth data
  std::vector<std::string> depth_sequence_names;
  for (auto& it : fs::directory_iterator(directory + directory_character + "depth" + directory_character))
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

  std::cout << "num frames: " << num_frames_ << std::endl
    << "video length: " << CurrentSequenceLength() << std::endl;

  // Load body data
  LoadBody();

  // Load robot trajectory
  LoadTrajectory();
}

void UtKinect::LoadBody()
{
  // Loading ground truth file
  auto body_filename = directory_ + directory_character + "joints" + directory_character + "joints_" + sequence_names_[current_sequence_] + ".txt";

  std::ifstream in(body_filename);

  joints_.resize(10000);
  joint_indices_.resize(10000);
  std::fill(joint_indices_.begin(), joint_indices_.end(), -1);

  int frame_index = 0;
  while (true)
  {
    frame_index = -1;

    in >> frame_index;

    if (frame_index == -1)
      break;

    joint_indices_[frame_index] = frame_index;

    for (int j = 0; j < num_joints_; j++)
    {
      in >> joints_[frame_index][j](0) >> joints_[frame_index][j](1) >> joints_[frame_index][j](2);

      // The y-direction is flipped
      joints_[frame_index][j](1) *= -1.;
    }
  }

  in.close();

  for (int i = 0; i < joint_indices_.size(); i++)
  {
    if (joint_indices_[i] == -1 && i > 0)
      joint_indices_[i] = joint_indices_[i - 1];
  }

  for (int i = joint_indices_.size() - 2; i >= 0; i--)
  {
    if (joint_indices_[i] == -1)
      joint_indices_[i] = joint_indices_[i + 1];
  }

  // Transform to Kinect space
  Matrix4d transform;
  transform.setIdentity();

  transform.block(0, 3, 3, 1) = Vector3d(0., 0., -0.5);

  // Convert joint coords to world space
  human_labels_.resize(joint_indices_.size());
  for (int i = 0; i < joint_indices_.size(); i++)
  {
    human_labels_[i] = nullptr;

    if (joint_indices_[i] == i)
    {
      human_labels_[i] = std::make_shared<HumanLabel>(human_model_);

      auto& human_label = *human_labels_[i];
      for (int j = 0; j < num_joints_; j++)
      {
        const auto& joint = joints_[i][j];

        if (joint.squaredNorm() < 1e-4)
          human_label.Position(j).setZero();
        else
        {
          // Transform from Qualysis space to Kinect space
          Vector4d joint_homo;
          joint_homo << joint, 1.;
          joint_homo = transform * joint_homo;

          // Convert milimeter to meter
          human_label.Position(j) = joint_homo.block(0, 0, 3, 1);
        }
      }
    }
  }

  body_loaded_ = true;
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

void UtKinect::SelectSequenceFrame(const std::string& name, const std::string& index)
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
    std::cout << "UtKinect: could not find sequence name \"" << name << "\"." << std::endl;
    return;
  }

  auto index_integer = std::atoi(index.c_str());
  for (int i = 0; i < num_frames_; i++)
  {
    if (rgb_image_indices_[i] == index_integer)
    {
      SelectFrame(i);
      return;
    }
  }

  std::cout << "UtKinect: could not find frame index \"" << index << "\"." << std::endl;
}

std::string UtKinect::GetCurrentSequenceName() const
{
  return sequence_names_[current_sequence_];
}

int UtKinect::FrameRate() const
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

int UtKinect::NumFrames() const
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

  auto filename = directory_ + directory_character + "depth" + directory_character + sequence_names_[current_sequence_] + directory_character + "depthImg" + std::to_string(cached_depth_image_index_) + ".xml";

  tinyxml2::XMLDocument doc;
  doc.LoadFile(filename.c_str());

  const char* str = doc.FirstChildElement("opencv_storage")
    ->FirstChildElement() // (depthImgXXX)
    ->FirstChildElement("data")
    ->GetText();

  std::istringstream iss(str);

  const int size = DepthWidth() * DepthHeight();
  cached_depth_image_.resize(size);
  int depth;

  // Flip y-axis direction
  for (int i = 0; i < DepthHeight(); i++)
  {
    for (int j = 0; j < DepthWidth(); j++)
    {
      const auto index = j + (DepthHeight() - i - 1) * DepthWidth();

      iss >> depth;

      // Distance unit change from x10000 to x1000
      cached_depth_image_[index] = depth / 10;
    }
  }

  return cached_depth_image_;
}

std::shared_ptr<HumanModel> UtKinect::GetHumanModel() const
{
  return human_model_;
}

std::shared_ptr<HumanLabel> UtKinect::GetHumanLabel() const
{
  // The color/depth's beginning indices are shifted to 0, but body indices are not
  // So lookup the color index and then the body index
  return human_labels_[joint_indices_[rgb_image_indices_[current_frame_]]];
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

void UtKinect::SelectFrame(int frame)
{
  if (frame < 0)
    current_frame_ = 0;

  else if (frame > num_frames_ - 1)
    current_frame_ = num_frames_ - 1;

  else
    current_frame_ = frame;
}

Trajectory UtKinect::GetTrajectory()
{
  return *trajectory_;
}

void UtKinect::LoadTrajectory()
{
  auto filename = directory_ + directory_character + "trajectories" + directory_character + "robot_trajectory_" + sequence_names_[current_sequence_] + ".txt";

  if (fs::exists(filename))
  {
    std::cout << "UtKinect: Loading Robot Trajectory" << std::endl;

    std::ifstream in(filename);

    int rows, cols;
    double time;
    in >> rows >> cols >> time;

    trajectory_ = std::make_unique<Trajectory>(rows, cols, time);

    for (int j = 0; j < cols; j++)
    {
      for (int i = 0; i < rows; i++)
        in >> (*trajectory_)(i, j);
    }

    in.close();
  }
  else
  {
    std::cout << "UtKinect: Robot trajectory file not found, loading initial trajectory" << std::endl;

    double duration = CurrentSequenceLength();
    int duration_ceil = static_cast<int>(std::ceil(duration)) + 1;

    // TODO: num_joints (the fetch robot model has 12 active joints)
    trajectory_ = std::make_unique<Trajectory>(12, duration_ceil, static_cast<double>(duration_ceil));
  }
}

void UtKinect::SaveTrajectory(Trajectory trajectory)
{
  std::string trajectory_filename = directory_ + directory_character + "trajectories" + directory_character + "robot_trajectory_" + sequence_names_[current_sequence_] + ".txt";
  Dataset::SaveTrajectory(trajectory, trajectory_filename);
}

double UtKinect::CurrentTime() const
{
  return static_cast<double>(current_frame_) / FrameRate();
}
}
