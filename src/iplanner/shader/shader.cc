#include "iplanner/shader/shader.h"

#include <stdexcept>
#include <fstream>

namespace iplanner
{
Shader::Shader()
{
}

Shader::Shader(const std::string& filename)
{
  Load(filename);
}

Shader::~Shader()
{
  if (generated_)
    glDeleteShader(id_);
}

Shader::Shader(Shader&& rhs)
{
  generated_ = rhs.generated_;
  type_ = rhs.type_;

  filename_ = std::move(rhs.filename_);
  source_ = std::move(rhs.source_);

  id_ = rhs.id_;

  // Reset basic type variables to default values
  rhs.generated_ = false;
  rhs.type_ = Type::UNDEFINED;
  rhs.id_ = 0;
}

Shader& Shader::operator = (Shader&& rhs)
{
  if (generated_)
    glDeleteShader(id_);

  generated_ = rhs.generated_;
  type_ = rhs.type_;

  filename_ = std::move(rhs.filename_);
  source_ = std::move(rhs.source_);

  id_ = rhs.id_;

  // Reset basic type variables to default values
  rhs.generated_ = false;
  rhs.type_ = Type::UNDEFINED;
  rhs.id_ = 0;

  return *this;
}

void Shader::Load(const std::string& filename)
{
  auto extension_pos = filename.find_last_of('.');
  if (extension_pos == std::string::npos)
    throw std::runtime_error("Shader: " + filename + " does not have a file extension.");

  auto extension = filename.substr(extension_pos);
  if (extension == ".vert")
    type_ = Type::VERTEX_SHADER;

  else if (extension == ".frag")
    type_ = Type::FRAGMENT_SHADER;

  else if (extension == ".geom")
    type_ = Type::GEOMETRY_SHADER;

  else
    throw std::runtime_error("Shader: Improper shader file extension " + extension);

  filename_ = filename;


  // Read file
  std::ifstream file(filename_);
  if (!file.is_open())
    throw std::runtime_error("Shader: Could not open file " + filename);

  file.seekg(0, std::ios::end);
  source_.resize(file.tellg());
  file.seekg(0);
  file.read(source_.data(), source_.size());

  file.close();


  // Generate GL shader
  switch (type_)
  {
  case Type::VERTEX_SHADER:
    id_ = glCreateShader(GL_VERTEX_SHADER);
    break;

  case Type::FRAGMENT_SHADER:
    id_ = glCreateShader(GL_FRAGMENT_SHADER);
    break;

  case Type::GEOMETRY_SHADER:
    id_ = glCreateShader(GL_GEOMETRY_SHADER);
    break;

  default:
    return;
  }

  generated_ = true;

  const char* source = source_.data();
  glShaderSource(id_, 1, &source, 0);
  glCompileShader(id_);

  int success;
  glGetShaderiv(id_, GL_COMPILE_STATUS, &success);

  if (!success)
  {
    char info_log[512];
    glGetShaderInfoLog(id_, 512, 0, info_log);
    throw std::runtime_error(std::string("Shader: Compilation failed.\n") + info_log);
  }
}
}
