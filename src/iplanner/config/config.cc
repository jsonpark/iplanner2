#include "iplanner/config/config.h"

#include <fstream>

#include "iplanner/utils/json.h"

namespace iplanner
{
Config::Config() = default;

Config::~Config() = default;

void Config::Load(const std::string& filename)
{
  Json json;

  std::ifstream in(filename);
  in >> json;
  in.close();

  for (auto dataset_directory_pair : json["dataset_directory"])
  {
    const auto& key = dataset_directory_pair.first;
    auto& dataset_directory_json = *dataset_directory_pair.second;

    dataset_directories_[key] = dataset_directory_json.Get<std::string>();
  }

  image_sample_save_directory_ = json["image_sample_save_directory"].Get<std::string>();
  auto& image_samples_json = json["image_samples"];
  for (int i = 0; i < image_samples_json.Size(); i++)
  {
    auto& image_sample_json = image_samples_json[i];

    ImageSample image_sample;
    image_sample.dataset = image_sample_json["dataset"].Get<std::string>();
    image_sample.sequence = image_sample_json["sequence"].Get<std::string>();
    image_sample.index = image_sample_json["index"].Get<std::string>();
    image_samples_.push_back(std::move(image_sample));
  }
}

void Config::PrintConfig()
{
}
}
