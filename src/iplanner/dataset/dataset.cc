#include "iplanner/dataset/dataset.h"

#include <iostream>
#include <fstream>

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

std::string Dataset::GetCurrentSequenceName() const
{
  return std::string("Dataset");
}

int Dataset::FrameRate() const
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

int Dataset::NumFrames() const
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

double Dataset::CurrentSequenceLength() const
{
  return static_cast<double>(NumFrames()) / FrameRate();
}

Trajectory Dataset::GetTrajectory()
{
  return Trajectory(9, 5, 1.);
}

void Dataset::SaveTrajectory(Trajectory trajectory)
{
  std::cout << "Dataset base class SaveTrajectory" << std::endl;
}

double Dataset::CurrentTime() const
{
  return 0.;
}

Trajectory Dataset::LoadTrajectory(const std::string& filename)
{
  std::ifstream in(filename);

  int rows, cols;
  double time;
  in >> rows >> cols >> time;

  Trajectory trajectory(rows, cols, time);

  for (int j = 0; j < cols; j++)
  {
    for (int i = 0; i < rows; i++)
      in >> trajectory(i, j);
  }

  in.close();

  return trajectory;
}

void Dataset::SaveTrajectory(Trajectory trajectory, const std::string& filename)
{
  // tab separated values

  std::ofstream out(filename);

  out << trajectory.Rows() << '\t' << trajectory.Cols() << '\t' << trajectory.Time() << std::endl;

  // Transpose the matrix so that each line represents a robot joint values at a frame
  for (int j = 0; j < trajectory.Cols(); j++)
  {
    for (int i = 0; i < trajectory.Rows(); i++)
      out << trajectory(i, j) << '\t';
    out << std::endl;
  }

  out.close();
}
}
