#ifndef IPLANNER_OBJECT_UNIFORM_H_
#define IPLANNER_OBJECT_UNIFORM_H_

#include <string>
#include <vector>
#include <unordered_map>

#include <glad/glad.h>

#include "iplanner/types.h"

namespace iplanner
{
class Program;

class UniformBuffer
{
public:
  enum class Type
  {
    INT,
    BOOL,
    FLOAT,
    VEC2,
    VEC3,
    VEC4,
    MAT2,
    MAT2X3,
    MAT2X4,
    MAT3X2,
    MAT3,
    MAT3X4,
    MAT4X2,
    MAT4X3,
    MAT4,
    UNDEFINED
  };

  static std::string TypeString(Type type);

private:
  struct Index
  {
    std::string name;
    int offset = 0;
    int base_alignment = 0;
    int size = 0;
    Type type = Type::UNDEFINED;
  };

public:
  class Value
  {
  public:
    Value() = delete;
    explicit Value(UniformBuffer& buffer, Index& index);
    ~Value();

    Value& operator = (int i);
    Value& operator = (bool i);
    Value& operator = (float v);
    Value& operator = (const Vector2f& v);
    Value& operator = (const Vector3f& v);
    Value& operator = (const Vector4f& v);
    Value& operator = (const Matrix2f& v);
    Value& operator = (const Matrix2x3f& v);
    Value& operator = (const Matrix2x4f& v);
    Value& operator = (const Matrix3x2f& v);
    Value& operator = (const Matrix3f& v);
    Value& operator = (const Matrix3x4f& v);
    Value& operator = (const Matrix4x2f& v);
    Value& operator = (const Matrix4x3f& v);
    Value& operator = (const Matrix4f& v);

  private:
    UniformBuffer& buffer_;
    Index& index_;
  };

  friend class Value;

private:
  static int binding_point_count_;

public:
  UniformBuffer();

  ~UniformBuffer();

  void AddMember(Type type, const std::string& name, int base_alignment = 0);
  void AddArrayMember(Type type, int count, const std::string& name);

  Value operator [] (const std::string& name);

  void Update();

  void PrintMembers();

  auto BindingPoint() const noexcept
  {
    return binding_point_;
  }

private:
  void Generate();

  void Bind();
  void Unbind();

  int ComputeNextOffset(int base_alignment);
  void Modify(int offset, int size, void* data);

  bool generated_ = false;
  bool need_update_ = true;

  GLuint id_ = 0;
  int binding_point_ = 0;

  std::vector<unsigned char> buffer_;

  std::vector<Index> members_;
  std::unordered_map<std::string, int> name_to_member_;
};
}

#endif // IPLANNER_OBJECT_UNIFORM_H_
