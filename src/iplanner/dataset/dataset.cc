#include "iplanner/dataset/dataset.h"

namespace iplanner
{
Dataset::Dataset() = default;

Dataset::~Dataset() = default;

int Dataset::NumSequences()
{
  return 1;
}

void Dataset::SelectSequence(int idx)
{
}

void Dataset::SelectSequence(const std::string& name)
{
}

void Dataset::SelectFrame(int index)
{
}

int Dataset::FrameRate()
{
  return 60;
}

int Dataset::RgbWidth()
{
  // Default Kinect v2
  return 1920;
}

int Dataset::RgbHeight()
{
  return 1080;
}

int Dataset::DepthWidth()
{
  return 512;
}

int Dataset::DepthHeight()
{
  return 424;
}

int Dataset::NumFrames()
{
  return 10;
}

std::vector<unsigned char> Dataset::GetRgbImage()
{
  return std::vector<unsigned char>(RgbWidth() * RgbHeight() * 3, 0);
}

std::vector<unsigned short> Dataset::GetDepthImage()
{
  return std::vector<unsigned short>(DepthWidth() * DepthHeight(), 0);
}

bool Dataset::PreviousSequence()
{
  return false;
}

bool Dataset::NextSequence()
{
  return false;
}

bool Dataset::PreviousFrame()
{
  return false;
}

bool Dataset::NextFrame()
{
  return false;
}

std::shared_ptr<HumanModel> Dataset::GetHumanModel() const
{
  return nullptr;
}

std::shared_ptr<HumanLabel> Dataset::GetHumanLabel() const
{
  return nullptr;
}
}
