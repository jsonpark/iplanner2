#ifndef IPLANNER_CONFIG_CONFIG_H_
#define IPLANNER_CONFIG_CONFIG_H_

#include <string>
#include <vector>
#include <unordered_map>

namespace iplanner
{
class Config
{
public:
  struct ImageSample
  {
    std::string dataset;
    std::string sequence;
    std::string index;
  };

public:
  Config();
  ~Config();

  void Load(const std::string& filename);

  const auto& GetVideoSaveDirectory() const
  {
    return video_save_direcotory_;
  }

  const auto& GetImageSampleSaveDirectory() const
  {
    return image_sample_save_directory_;
  }

  const auto& GetImageSamples() const
  {
    return image_samples_;
  }

  const std::string& GetDatasetDirectory(const std::string& name)
  {
    if (dataset_directories_.find(name) == dataset_directories_.cend())
      return "";

    return dataset_directories_[name];
  }

  void PrintConfig();

private:
  std::unordered_map<std::string, std::string> dataset_directories_;

  std::string image_sample_save_directory_;
  std::vector<ImageSample> image_samples_;
  std::string video_save_direcotory_;
};
}

#endif // IPLANNER_CONFIG_CONFIG_H_
