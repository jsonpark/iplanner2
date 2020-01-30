#ifndef IPLANNER_DATASET_DATASET_H_
#define IPLANNER_DATASET_DATASET_H_

#include <string>
#include <vector>
#include <memory>

#include "iplanner/human/human_label.h"
#include "iplanner/plan/trajectory.h"

namespace iplanner
{
class Dataset
{
public:
  Dataset();
  virtual ~Dataset();

  virtual int NumSequences();
  virtual void SelectSequence(int idx);
  virtual void SelectSequence(const std::string& name);
  virtual void SelectFrame(int index);

  virtual int FrameRate() const;
  virtual int RgbWidth();
  virtual int RgbHeight();
  virtual int DepthWidth();
  virtual int DepthHeight();

  virtual int NumFrames() const;
  virtual std::vector<unsigned char> GetRgbImage();
  virtual std::vector<unsigned short> GetDepthImage();

  virtual std::shared_ptr<HumanModel> GetHumanModel() const;
  virtual std::shared_ptr<HumanLabel> GetHumanLabel() const;

  virtual bool PreviousSequence();
  virtual bool NextSequence();
  virtual bool PreviousFrame();
  virtual bool NextFrame();

  virtual Trajectory GetTrajectory();
  virtual void SaveTrajectory(Trajectory trajectory);

  virtual double CurrentSequenceLength() const;

private:
};
}

#endif // IPLANNER_DATASET_DATASET_H_
