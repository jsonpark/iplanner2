#include "iplanner/object/uniform_buffer.h"

#include <iostream>
#include <iomanip>

#include "iplanner/shader/program.h"

namespace iplanner
{
std::string UniformBuffer::TypeString(Type type)
{
  switch (type)
  {
  case Type::BOOL:
    return "bool";

  case Type::INT:
    return "int";

  case Type::FLOAT:
    return "float";

  case Type::VEC2:
    return "vec2";

  case Type::VEC3:
    return "vec3";

  case Type::VEC4:
    return "vec4";

  case Type::MAT2:
    return "mat2";

  case Type::MAT2X3:
    return "mat2x3";

  case Type::MAT2X4:
    return "mat2x4";

  case Type::MAT3X2:
    return "mat3x2";

  case Type::MAT3:
    return "mat3";

  case Type::MAT3X4:
    return "mat3x4";

  case Type::MAT4X2:
    return "mat4x2";

  case Type::MAT4X3:
    return "mat4x3";

  case Type::MAT4:
    return "mat4";

  default:
    return "(undefined)";
  }
}

//
// Value
//
UniformBuffer::Value::Value(UniformBuffer& buffer, Index& index)
  : buffer_(buffer), index_(index)
{
}

UniformBuffer::Value::~Value() = default;

UniformBuffer::Value& UniformBuffer::Value::operator = (int i)
{
  if (index_.type != Type::INT &&
    index_.type != Type::BOOL)
    throw std::runtime_error("Uniform Buffer Value: tried to insert integer into a type id " + std::to_string(static_cast<int>(index_.type)));

  buffer_.Modify(index_.offset, index_.size, &i);
}

UniformBuffer::Value& UniformBuffer::Value::operator = (bool i)
{
  if (index_.type != Type::BOOL)
    throw std::runtime_error("Uniform Buffer Value: tried to insert boolean into a type id " + std::to_string(static_cast<int>(index_.type)));

  int i_byte = i;
  buffer_.Modify(index_.offset, index_.size, &i_byte);
}

UniformBuffer::Value& UniformBuffer::Value::operator = (float v)
{
  if (index_.type != Type::FLOAT)
    throw std::runtime_error("Uniform Buffer Value: tried to insert float into a type id " + std::to_string(static_cast<int>(index_.type)));

  buffer_.Modify(index_.offset, index_.size, &v);
}

UniformBuffer::Value& UniformBuffer::Value::operator = (const Vector2f& v)
{
  if (index_.type != Type::VEC2)
    throw std::runtime_error("Uniform Buffer Value: tried to insert vec2 into a type id " + std::to_string(static_cast<int>(index_.type)));

  buffer_.Modify(index_.offset, index_.size, (void *)v.data());
}

UniformBuffer::Value& UniformBuffer::Value::operator = (const Vector3f& v)
{
  if (index_.type != Type::VEC3)
    throw std::runtime_error("Uniform Buffer Value: tried to insert vec3 into a type id " + std::to_string(static_cast<int>(index_.type)));

  buffer_.Modify(index_.offset, index_.size, (void*)v.data());
}

UniformBuffer::Value& UniformBuffer::Value::operator = (const Vector4f& v)
{
  if (index_.type != Type::VEC4)
    throw std::runtime_error("Uniform Buffer Value: tried to insert vec4 into a type id " + std::to_string(static_cast<int>(index_.type)));

  buffer_.Modify(index_.offset, index_.size, (void*)v.data());
}

UniformBuffer::Value& UniformBuffer::Value::operator = (const Matrix2f& m)
{
  if (index_.type != Type::MAT2)
    throw std::runtime_error("Uniform Buffer Value: tried to insert mat2 into a type id " + std::to_string(static_cast<int>(index_.type)));

  float v[] = {
    m(0, 0), m(1, 0), 0.f, 0.f,
    m(0, 1), m(1, 1), 0.f, 0.f,
  };
  buffer_.Modify(index_.offset, index_.size, v);
}

UniformBuffer::Value& UniformBuffer::Value::operator = (const Matrix2x3f& m)
{
  if (index_.type != Type::MAT2X3)
    throw std::runtime_error("Uniform Buffer Value: tried to insert mat2x3 into a type id " + std::to_string(static_cast<int>(index_.type)));

  float v[] = {
    m(0, 0), m(1, 0), 0.f, 0.f,
    m(0, 1), m(1, 1), 0.f, 0.f,
    m(0, 2), m(1, 2), 0.f, 0.f,
  };
  buffer_.Modify(index_.offset, index_.size, v);
}

UniformBuffer::Value& UniformBuffer::Value::operator = (const Matrix2x4f& m)
{
  if (index_.type != Type::MAT2X4)
    throw std::runtime_error("Uniform Buffer Value: tried to insert mat2x4 into a type id " + std::to_string(static_cast<int>(index_.type)));

  float v[] = {
    m(0, 0), m(1, 0), 0.f, 0.f,
    m(0, 1), m(1, 1), 0.f, 0.f,
    m(0, 2), m(1, 2), 0.f, 0.f,
    m(0, 3), m(1, 3), 0.f, 0.f,
  };
  buffer_.Modify(index_.offset, index_.size, v);
}

UniformBuffer::Value& UniformBuffer::Value::operator = (const Matrix3x2f& m)
{
  if (index_.type != Type::MAT3X2)
    throw std::runtime_error("Uniform Buffer Value: tried to insert mat3x2 into a type id " + std::to_string(static_cast<int>(index_.type)));

  float v[] = {
    m(0, 0), m(1, 0), m(2, 0), 0.f,
    m(0, 1), m(1, 1), m(2, 1), 0.f,
  };
  buffer_.Modify(index_.offset, index_.size, v);
}

UniformBuffer::Value& UniformBuffer::Value::operator = (const Matrix3f& m)
{
  if (index_.type != Type::MAT3)
    throw std::runtime_error("Uniform Buffer Value: tried to insert mat3 into a type id " + std::to_string(static_cast<int>(index_.type)));

  float v[] = {
    m(0, 0), m(1, 0), m(2, 0), 0.f,
    m(0, 1), m(1, 1), m(2, 1), 0.f,
    m(0, 2), m(1, 2), m(2, 2), 0.f,
  };
  buffer_.Modify(index_.offset, index_.size, v);
}

UniformBuffer::Value& UniformBuffer::Value::operator = (const Matrix3x4f& m)
{
  if (index_.type != Type::MAT3X4)
    throw std::runtime_error("Uniform Buffer Value: tried to insert mat3x4 into a type id " + std::to_string(static_cast<int>(index_.type)));

  float v[] = {
    m(0, 0), m(1, 0), m(2, 0), 0.f,
    m(0, 1), m(1, 1), m(2, 1), 0.f,
    m(0, 2), m(1, 2), m(2, 2), 0.f,
    m(0, 3), m(1, 3), m(2, 3), 0.f,
  };
  buffer_.Modify(index_.offset, index_.size, v);
}

UniformBuffer::Value& UniformBuffer::Value::operator = (const Matrix4x2f& m)
{
  if (index_.type != Type::MAT4X2)
    throw std::runtime_error("Uniform Buffer Value: tried to insert mat4x2 into a type id " + std::to_string(static_cast<int>(index_.type)));

  buffer_.Modify(index_.offset, index_.size, (void*)m.data());
}

UniformBuffer::Value& UniformBuffer::Value::operator = (const Matrix4x3f& m)
{
  if (index_.type != Type::MAT4X3)
    throw std::runtime_error("Uniform Buffer Value: tried to insert mat4x3 into a type id " + std::to_string(static_cast<int>(index_.type)));

  buffer_.Modify(index_.offset, index_.size, (void*)m.data());
}

UniformBuffer::Value& UniformBuffer::Value::operator = (const Matrix4f& m)
{
  if (index_.type != Type::MAT4)
    throw std::runtime_error("Uniform Buffer Value: tried to insert mat4 into a type id " + std::to_string(static_cast<int>(index_.type)));

  buffer_.Modify(index_.offset, index_.size, (void*)m.data());
}


//
// UniformBuffer
//

int UniformBuffer::binding_point_count_ = 0;

UniformBuffer::UniformBuffer()
  : binding_point_(binding_point_count_++)
{
}

UniformBuffer::~UniformBuffer()
{
  if (generated_)
    glDeleteBuffers(1, &id_);
}

void UniformBuffer::AddMember(Type type, const std::string& name, int base_alignment)
{
  Index index;
  index.type = type;
  index.name = name;
  
  if (base_alignment != 0)
    index.base_alignment = base_alignment;

  else
  {
    switch (type)
    {
    case Type::FLOAT:
    case Type::INT:
    case Type::BOOL:
      index.base_alignment = 4;
      break;

    case Type::VEC2:
      index.base_alignment = 8;
      break;

    case Type::VEC3:
    case Type::VEC4:
      index.base_alignment = 16;
      break;

    case Type::MAT2:
    case Type::MAT3X2:
    case Type::MAT4X2:
    case Type::MAT2X3:
    case Type::MAT3:
    case Type::MAT4X3:
    case Type::MAT2X4:
    case Type::MAT3X4:
    case Type::MAT4:
      index.base_alignment = 16;
      break;

    default:
      return;
    }
  }

  // Size
  switch (type)
  {
  case Type::FLOAT:
  case Type::INT:
  case Type::BOOL:
    index.size = 4;
    break;

  case Type::VEC2:
    index.size = 8;
    break;

  case Type::VEC3:
    index.size = 12;
    break;

  case Type::VEC4:
    index.size = 16;
    break;

  case Type::MAT2:
  case Type::MAT3X2:
  case Type::MAT4X2:
    index.size = 32;
    break;

  case Type::MAT2X3:
  case Type::MAT3:
  case Type::MAT4X3:
    index.size = 48;
    break;

  case Type::MAT2X4:
  case Type::MAT3X4:
  case Type::MAT4:
    index.size = 64;
    break;

  default:
    return;
  }

  if (members_.empty())
    index.offset = 0;

  else
    index.offset = ComputeNextOffset(index.base_alignment);

  name_to_member_[index.name] = members_.size();
  members_.push_back(index);

  buffer_.resize(index.offset + index.size, 0);

  need_update_ = true;

  // TODO: add members of an array of block
}

void UniformBuffer::AddArrayMember(Type type, int count, const std::string& name)
{
  // Base alignment is rounded up to the base alignment of a vec4.
  int base_alignment = 0;

  switch (type)
  {
    // scalar
  case Type::BOOL:
  case Type::INT:
  case Type::FLOAT:
    base_alignment = 16;
    break;

    // vector
  case Type::VEC2:
  case Type::VEC3:
  case Type::VEC4:
    base_alignment = 16;
    break;

    // Matrices are considered to have vec4 columns, so the base alignment is not needed to be rounded up.
  }

  for (int i = 0; i < count; i++)
    AddMember(type, name + "[" + std::to_string(i) + "]", base_alignment);
}

UniformBuffer::Value UniformBuffer::operator [] (const std::string& name)
{
  Value value(*this, members_[name_to_member_[name]]);
  return value;
}

int UniformBuffer::ComputeNextOffset(int base_alignment)
{
  const auto& last_member = members_.back();

  // size is multiple of base_alignment, or
  // size may be smaller than the base alignment when the base alignment is rounded up to it of vec4
  auto compact = last_member.offset + std::max(last_member.size, last_member.base_alignment);

  return (compact + base_alignment - 1) / base_alignment * base_alignment;
}

void UniformBuffer::Modify(int offset, int size, void* data)
{
  auto ptr = (unsigned char*)data;
  for (int i = 0; i < size; i++)
    buffer_[offset + i] = ptr[i];

  need_update_ = true;
}

void UniformBuffer::Update()
{
  if (need_update_)
  {
    if (!generated_)
      Generate();

    Bind();
    glBufferSubData(GL_UNIFORM_BUFFER, 0, buffer_.size(), buffer_.data());
    Unbind();

    need_update_ = false;
  }
}

void UniformBuffer::Generate()
{
  glGenBuffers(1, &id_);

  Bind();

  glBufferData(GL_UNIFORM_BUFFER, buffer_.size(), 0, GL_STATIC_DRAW);
  glBindBufferBase(GL_UNIFORM_BUFFER, binding_point_, id_);

  Unbind();

  generated_ = true;
}

void UniformBuffer::Bind()
{
  glBindBuffer(GL_UNIFORM_BUFFER, id_);
}

void UniformBuffer::Unbind()
{
  glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void UniformBuffer::PrintMembers()
{
  std::cout << "Uniform buffer block (binding point " << binding_point_ << ")" << std::endl
    << " id   base offset size type   name" << std::endl;
  for (int i = 0; i < members_.size(); i++)
  {
    const auto& member = members_[i];

    std::cout << std::right << std::setw(3) << i << " : "
      << std::setw(4) << member.base_alignment << " "
      << std::setw(6) << member.offset << " "
      << std::setw(4) << member.size << " "
      << std::left << std::setw(6) << TypeString(member.type) << " " << member.name << std::endl;
  }
}
}
