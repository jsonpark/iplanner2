#ifndef IPLANNER_ROBOT_ROBOT_MODEL_LOADER_H_
#define IPLANNER_ROBOT_ROBOT_MODEL_LOADER_H_

#include <memory>
#include <string>

#include <tinyxml2/tinyxml2.h>

#include "iplanner/robot/robot_model.h"
#include "iplanner/robot/robot_link.h"
#include "iplanner/robot/robot_joint.h"

namespace iplanner
{
class RobotModelLoader
{
private:
#ifdef _WIN32
  static constexpr char directory_char_ = '\\';
#else
  static constexpr char directory_char_ = '/';
#endif

public:
  RobotModelLoader();
  ~RobotModelLoader();

  void SetPackageDirectory(const std::string& package_directory);

  std::shared_ptr<RobotModel> Load(const std::string& filename);

private:
  RobotLink::Geometry GetGeometryFromElement(tinyxml2::XMLElement* element);

  std::string ConvertFilenamePackageDirectory(const std::string& filename);

  std::string package_directory_;
};
}

#endif // IPLANNER_ROBOT_ROBOT_MODEL_LOADER_H_
