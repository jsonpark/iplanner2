#ifndef IPLANNER_DATASET_DATASET_H_
#define IPLANNER_DATASET_DATASET_H_

#include <string>
#include <vector>

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

  virtual int FrameRate();
  virtual int RgbWidth();
  virtual int RgbHeight();
  virtual int DepthWidth();
  virtual int DepthHeight();

  virtual int NumFrames();
  virtual std::vector<unsigned char> GetRgbImage();
  virtual std::vector<unsigned short> GetDepthImage();

  virtual bool PreviousSequence();
  virtual bool NextSequence();
  virtual bool PreviousFrame();
  virtual bool NextFrame();

private:
};
}

#endif // IPLANNER_DATASET_DATASET_H_
