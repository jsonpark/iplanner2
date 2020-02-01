#include "iplanner/dataset/dataset_wnp.h"

#ifdef _WIN32
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

#include <iostream>
#include <fstream>
#include <algorithm>

#include <stb/stb_image.h>
//#include <mat.h>

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

// Extract file index XXXX from filename format "XXXX.jpg" and "XXXX.xml"
int ExtractFileIndex(const std::string& filename)
{
  int len = filename.length();

  // Length is 4 (index) + 4 (extension) = 8
  if (len == 8 && 
    (filename.substr(4) == ".jpg" || filename.substr(4) == ".mat"))
    return std::stoi(filename.substr(0, 4));

  return 10000;
}

std::string ZeroPrependedString(int length, int number)
{
  auto s = std::to_string(number);
  if (s.length() < 4)
    return std::string(4 - s.length(), '0') + s;
  return s;
}
}

const std::vector<std::string> Wnp::scene_names_ =
{
  "office",
  "kitchen1",
  "kitchen2",
};

Wnp::Wnp(const std::string& directory)
  : directory_(directory)
{
  // Load matlab module
  // TODO: keep the matlab module singleton
  /*
  const auto& matlabs = matlab::engine::findMATLAB();
  std::cout << "Active sharable matlab engines:" << std::endl;
  for (const auto& matlab : matlabs)
    std::cout << "* " << matlab::engine::convertUTF16StringToUTF8String(matlab) << std::endl;
  std::cout << "Starting matlab module async" << std::endl;
  matlab_future_ = matlab::engine::connectMATLABAsync();
  */
  

  // Human model
  std::vector<std::string> joint_names
  {
    "SpineBase",
    "SpineMid",
    "Neck",
    "Head",
    "ShoulderLeft",
    "ElbowLeft",
    "WristLeft",
    "HandLeft",
    "ShoulderRight",
    "ElbowRight",
    "WristRight",
    "HandRight",
    "HipLeft",
    "KneeLeft",
    "AnkleLeft",
    "FootLeft",
    "HipRight",
    "KneeRight",
    "AnkleRight",
    "FootRight",
    "SpineShoulder",
    "HandTipLeft",
    "ThumbLeft",
    "HandTipRight",
    "ThumbRight",
  };

  human_model_ = std::make_shared<HumanModel>(std::move(joint_names));


  // Get sequence names
  for (const auto& scene_name : scene_names_)
  {
    for (auto& it : fs::directory_iterator(directory + directory_character + scene_name + directory_character))
    {
      const auto& path = it.path();
      sequence_names_.push_back(scene_name + directory_character + path.filename().string());
    }
  }
  std::sort(sequence_names_.begin(), sequence_names_.end());

  SelectSequence(0);
}

Wnp::~Wnp()
{
}

int Wnp::NumSequences()
{
  return sequence_names_.size();
}

void Wnp::SelectSequence(int idx)
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

  // Load trajectory data
  LoadTrajectory();
}

void Wnp::LoadBody()
{
  // Loading .mat file using matlab module (slow)
  //LoadBodyFromMatFile();

  // Loading .struct file in custom format (fast)
  LoadBodyFromStructFile();
}

void Wnp::LoadBodyFromStructFile()
{
  auto body_filename = directory_ + directory_character + sequence_names_[current_sequence_] + directory_character + "body.struct";

  std::ifstream in(body_filename);

  char buffer[256];
  int num_frames;
  in >> buffer >> num_frames;
  if (num_frames != num_frames_)
  {
    std::cout << "Dataset WNP error: number of frames (" << num_frames << ") in body file is different from one inferred from color/depth images (" << num_frames_ << ")" << std::endl;
    return;
  }

  is_body_tracked_.resize(num_frames);
  joints_.resize(num_frames);

  const int num_joints = human_model_->NumJoints();
  for (int i = 0; i < num_frames; i++)
  {
    // frame %d isBodyTracked: %d
    int frame_id;
    int is_body_tracked;
    in >> buffer >> frame_id >> buffer >> is_body_tracked;

    if (is_body_tracked)
    {
      is_body_tracked_[i] = true;

      Joints& joints = joints_[i];

      for (int j = 0; j < num_joints; j++)
      {
        // joint %d
        int joint_id;
        in >> buffer >> joint_id;

        //  trackingState: %d
        //  camera: %lf %lf %lf
        //  color: %lf %lf
        //  depth: %lf %lf
        //  rotation: %lf %lf %lf %lf
        //  pcloud: %lf %lf %lf
        Joint& joint = joints[j];
        in >> buffer >> joint.tracking_state
          >> buffer >> joint.camera(0) >> joint.camera(1) >> joint.camera(2)
          >> buffer >> joint.color(0) >> joint.color(1)
          >> buffer >> joint.depth(0) >> joint.depth(1)
          >> buffer >> joint.rotation.x() >> joint.rotation.y() >> joint.rotation.z() >> joint.rotation.w()
          >> buffer >> joint.pcloud(0) >> joint.pcloud(1) >> joint.pcloud(2);
        joint.rotation.normalize();
      }
    }

    else
      is_body_tracked_[i] = false;
  }

  in.close();

  // Convert joint coords to world space
  human_labels_.resize(num_frames);
  for (int i = 0; i < num_frames; i++)
  {
    if (is_body_tracked_[i])
    {
      human_labels_[i] = std::make_shared<HumanLabel>(human_model_);

      auto& human_label = *human_labels_[i];
      for (int j = 0; j < num_joints; j++)
      {
        const auto& joint = joints_[i][j];

        // Convert milimeter to meter
        human_label.TrackingState(j) = joint.tracking_state;
        human_label.Position(j) = joint.pcloud / 1000.;
      }
    }

    else
      human_labels_[i] = nullptr;
  }
}

void Wnp::LoadBodyFromMatFile()
{
  /*
  // Read body data, assuming the number of body frames matches with the number of color/depth frames
  if (matlab_ == nullptr && matlab_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    matlab_ = matlab_future_.get();

  if (matlab_ != nullptr)
  {
    body_loaded_ = true;

    auto body_filename = directory_ + directory_character + sequence_names_[current_sequence_] + directory_character + "body.mat";

    matlab_->eval(u"load " + matlab::engine::convertUTF8StringToUTF16String(body_filename));

    matlab::data::Array body = matlab_->getVariable("body");
    auto body_dim = body.getDimensions();

    for (int i = 0; i < body_dim[0]; i++)
    {
      int body_index = -1;
      for (int j = 0; j < body_dim[1]; j++)
      {
        matlab::data::StructArray body_ij = body[i][j];
        matlab::data::TypedArray<double> is_body_tracked = body_ij[0]["isBodyTracked"];

        if (is_body_tracked[0] == 1)
        {
          body_index = j;
          break;
        }
      }

      if (body_index == -1)
      {
        std::cout << "skipped " << i << "-th frame as no humans are tracked" << std::endl;
        continue;
      }

      matlab::data::StructArray body_i = body[i][body_index];
      matlab::data::Array joints = body_i[0]["joints"];

      for (int j = 0; j < joints.getDimensions()[1]; j++)
      {
        matlab::data::StructArray joint = joints[0][j];

        matlab::data::TypedArray<double> tracking_state = joint[0]["trackingState"];
        matlab::data::TypedArray<double> camera = joint[0]["camera"];
        matlab::data::TypedArray<double> color = joint[0]["color"];
        matlab::data::TypedArray<double> depth = joint[0]["depth"];
        matlab::data::TypedArray<double> rotation = joint[0]["rotation"];
        matlab::data::TypedArray<double> pcloud = joint[0]["pcloud"];

        std::cout << "trackingState: " << tracking_state[0] << "]" << std::endl;
        std::cout << "camera: [" << camera[0] << "; " << camera[1] << "; " << camera[2] << "]" << std::endl;
        std::cout << "color: [" << color[0] << "; " << color [1] << "]" << std::endl;
        std::cout << "depth: [" << depth[0] << "; " << depth [1] << "]" << std::endl;
        std::cout << "rotation: [" << rotation[0] << "; " << rotation[1] << "; " << rotation[2] << "; " << rotation [3] << "]" << std::endl;
        std::cout << "pcloud: [" << pcloud[0] << "; " << pcloud[1] << "; " << pcloud[2] << "]" << std::endl;
      }
    }

    std::cout << "clear" << std::endl;

    matlab_->eval(u"clear body");
  }
  */
}

void Wnp::LoadTrajectory()
{
  auto filename = directory_ + directory_character + sequence_names_[current_sequence_] + directory_character + "robot_trajectory.txt";

  if (fs::exists(filename))
  {
    std::cout << "Wnp: Loading Robot Trajectory" << std::endl;
    trajectory_ = std::make_unique<Trajectory>(Dataset::LoadTrajectory(filename));

    for (int j = 0; j < trajectory_->Cols(); j++)
    {
      for (int i = 0; i < trajectory_->Rows(); i++)
      {
        std::cout << trajectory_->operator()(i, j) << ' ';
      }
      std::cout << std::endl;
    }
  }
  else
  {
    std::cout << "Wnp: Robot trajectory file not found, loading initial trajectory" << std::endl;

    double duration = CurrentSequenceLength();
    int duration_ceil = static_cast<int>(std::ceil(duration)) + 1;

    // TODO: num_joints (the fetch robot model has 12 active joints)
    trajectory_ = std::make_unique<Trajectory>(12, duration_ceil, static_cast<double>(duration_ceil));
  }
}

void Wnp::SelectSequence(const std::string& name)
{
  for (int i = 0; i < sequence_names_.size(); i++)
  {
    if (sequence_names_[i] == name)
    {
      SelectSequence(i);
      return;
    }
  }

  std::cout << "Wnp: could not find sequence name \"" << name << "\"." << std::endl;
}

void Wnp::SelectSequenceFrame(const std::string& name, const std::string& index)
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
    std::cout << "Wnp: could not find sequence name \"" << name << "\"." << std::endl;
    return;
  }

  for (int i = 0; i < num_frames_; i++)
  {
    if (ZeroPrependedString(4, rgb_image_indices_[i]) == index)
    {
      SelectFrame(i);
      return;
    }
  }

  std::cout << "Wnp: could not find frame index \"" << index << "\"." << std::endl;
}

std::string Wnp::GetCurrentSequenceName() const
{
  return sequence_names_[current_sequence_];
}

int Wnp::FrameRate() const
{
  return 6;
}

int Wnp::RgbWidth()
{
  return 1920;
}

int Wnp::RgbHeight()
{
  return 1080;
}

int Wnp::DepthWidth()
{
  return 512;
}

int Wnp::DepthHeight()
{
  return 424;
}

int Wnp::NumFrames() const
{
  return num_frames_;
}

std::vector<unsigned char> Wnp::GetRgbImage()
{
  if (cached_rgb_image_index_ == rgb_image_indices_[current_frame_])
    return cached_rgb_image_;

  cached_rgb_image_index_ = rgb_image_indices_[current_frame_];

  auto filename = directory_ + directory_character + sequence_names_[current_sequence_] + directory_character + "rgbjpg" + directory_character + ZeroPrependedString(4, cached_rgb_image_index_) + ".jpg";
  int width, height, components;
  stbi_set_flip_vertically_on_load(true);
  unsigned char* data = stbi_load(filename.c_str(), &width, &height, &components, 0);
  cached_rgb_image_ = std::vector<unsigned char>(data, data + static_cast<std::uint64_t>(width) * height * components);
  stbi_image_free(data);

  return cached_rgb_image_;
}

std::vector<unsigned short> Wnp::GetDepthImage()
{
  if (cached_depth_image_index_ == depth_image_indices_[current_frame_])
    return cached_depth_image_;

  cached_depth_image_index_ = depth_image_indices_[current_frame_];

  cached_depth_image_.resize(DepthWidth() * DepthHeight());

  auto filename = directory_ + directory_character + sequence_names_[current_sequence_] + directory_character + "depth_raw" + directory_character + ZeroPrependedString(4, cached_depth_image_index_) + ".raw";
  std::ifstream in(filename, std::ios::in | std::ios::binary);

  auto ptr = reinterpret_cast<char*>(cached_depth_image_.data());
  for (int i = 0; i < DepthHeight(); i++)
  {
    int index = (DepthHeight() - i - 1) * DepthWidth();

    // Type of ptr is (char*), whose index would have been counted as the type were (unsigned short*).
    in.read(ptr + index * sizeof(unsigned short), DepthWidth() * sizeof(unsigned short));
  }

  in.close();

  /*
  for (int i = 0; i < cached_depth_image_.size(); i++)
    cached_depth_image_[i] = 0;
    */

  /*
  auto filename = directory_ + directory_character + sequence_names_[current_sequence_] + directory_character + "depth" + directory_character + ZeroPrependedString(4, cached_depth_image_index_) + ".mat";

  MATFile* mat;
  mat = matOpen(filename.c_str(), "r");

  mxArray* arr = matGetVariable(mat, "depth");
  double* ptr = mxGetPr(arr);
  cached_depth_image_.resize(DepthWidth() * DepthHeight());
  for (int i = 0; i < cached_depth_image_.size(); i++)
    cached_depth_image_[i] = static_cast<unsigned short>(ptr[i]);
  mxDestroyArray(arr);

  matClose(mat);
  */

  /*
  if (matlab_ == nullptr && matlab_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    matlab_ = matlab_future_.get();

  if (matlab_ != nullptr)
  {
    auto depth_filename = directory_ + directory_character + sequence_names_[current_sequence_] + directory_character + "depth" + directory_character + ZeroPrependedString(4, cached_depth_image_index_) + ".mat";

    matlab_->eval(u"load " + matlab::engine::convertUTF8StringToUTF16String(depth_filename));

    matlab::data::Array depth = matlab_->getVariable("depth");
    auto depth_dim = depth.getDimensions();

    int depth_index = 0;
    for (int i = 0; i < depth_dim[1]; i++)
    {
      for (int j = 0; j < depth_dim[0]; j++)
        cached_depth_image_[depth_index++] = depth[j][i];
    }

    //matlab_->eval(u"clear depth");
  }
  */

  return cached_depth_image_;
}

std::shared_ptr<HumanModel> Wnp::GetHumanModel() const
{
  return human_model_;
}

std::shared_ptr<HumanLabel> Wnp::GetHumanLabel() const
{
  return human_labels_[current_frame_];
}

bool Wnp::PreviousSequence()
{
  if (current_sequence_ > 0)
  {
    SelectSequence(current_sequence_ - 1);
    return true;
  }

  return false;
}

bool Wnp::NextSequence()
{
  if (current_sequence_ < NumSequences() - 1)
  {
    SelectSequence(current_sequence_ + 1);
    return true;
  }

  return false;
}

bool Wnp::PreviousFrame()
{
  if (current_frame_ > 0)
  {
    current_frame_--;
    return true;
  }

  return false;
}

bool Wnp::NextFrame()
{
  if (current_frame_ < NumFrames() - 1)
  {
    current_frame_++;
    return true;
  }

  return false;
}

void Wnp::SelectFrame(int frame)
{
  if (frame < 0)
    current_frame_ = 0;

  else if (frame > num_frames_ - 1)
    current_frame_ = num_frames_ - 1;

  else
    current_frame_ = frame;
}

Trajectory Wnp::GetTrajectory()
{
  if (trajectory_ == nullptr)
    return Dataset::GetTrajectory();

  return *trajectory_;
}

void Wnp::SaveTrajectory(Trajectory trajectory)
{
  std::string filename = directory_ + directory_character + sequence_names_[current_sequence_] + directory_character + "robot_trajectory.txt";
  Dataset::SaveTrajectory(trajectory, filename);
}

double Wnp::CurrentTime() const
{
  return static_cast<double>(current_frame_) / FrameRate();
}
}
