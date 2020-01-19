#ifndef IPLANNER_SHADER_PROGRAM_H_
#define IPLANNER_SHADER_PROGRAM_H_

#include <initializer_list>
#include <vector>

#include "iplanner/shader/shader.h"
#include "iplanner/types.h"
#include "iplanner/object/uniform_buffer.h"

namespace iplanner
{
class Program
{
public:
  // Empty shader program
  Program();

  // Load shaders with extensions .vert, .frag, .geom
  Program(const std::string& path, const std::string& name);

  ~Program();

  // Copy constructors are deleted
  Program(const Program& rhs) = delete;

  Program& operator = (const Program& rhs) = delete;

  // Move constructors
  Program(Program&& rhs);

  Program& operator = (Program&& rhs);

  auto Id() const noexcept
  {
    return id_;
  }

  void Use();

  void Uniform1f(const std::string& name, float v);
  void Uniform1i(const std::string& name, int i);
  void Uniform2f(const std::string& name, const Vector2f& v);
  void Uniform3f(const std::string& name, const Vector3f& v);
  void Uniform4f(const std::string& name, const Vector4f& v);
  void UniformMatrix3f(const std::string& name, const Matrix3f& m);
  void UniformMatrix4f(const std::string& name, const Matrix4f& m);

  void BindUniformBuffer(int index, std::shared_ptr<UniformBuffer> buffer);
  void BindUniformBuffer(int index, const UniformBuffer& buffer);

private:
  void Load(const std::string& path, const std::string& name);

  bool generated_ = false;

  std::vector<Shader> shaders_;

  GLuint id_;
};
}

#endif // IPLANNER_SHADER_PROGRAM_H_
