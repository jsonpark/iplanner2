#ifndef IPLANNER_OBJECT_BUFFER_H_
#define IPLANNER_OBJECT_BUFFER_H_

#include <iostream>
#include <vector>
#include <algorithm>

#include <glad/glad.h>

namespace iplanner
{
enum class BufferType
{
  ARRAY_BUFFER,
  ELEMENT_ARRAY_BUFFER,
  UNDEFINED,
};

enum class BufferUsage
{
  STATIC_DRAW,
  DYNAMIC_DRAW,
  STREAM_DRAW,
  UNDEFINED,
};

namespace internal
{
// Type to GL enum
constexpr GLenum ToGlType(BufferType type)
{
  switch (type)
  {
  case BufferType::ARRAY_BUFFER:
    return GL_ARRAY_BUFFER;

  case BufferType::ELEMENT_ARRAY_BUFFER:
    return GL_ELEMENT_ARRAY_BUFFER;

  default:
    return 0;
  }
}

// Usage to Gl enum
constexpr GLenum ToGlUsage(BufferUsage usage)
{
  switch (usage)
  {
  case BufferUsage::STATIC_DRAW:
    return GL_STATIC_DRAW;

  case BufferUsage::DYNAMIC_DRAW:
    return GL_DYNAMIC_DRAW;

  case BufferUsage::STREAM_DRAW:
    return GL_STREAM_DRAW;

  default:
    return 0;
  }
}

// Data type to GL enum
template <typename T>
constexpr GLenum ToGlDataType()
{
  return 0;
}

template <>
constexpr GLenum ToGlDataType<float>()
{
  return GL_FLOAT;
}

template <>
constexpr GLenum ToGlDataType<int>()
{
  return GL_INT;
}

template <>
constexpr GLenum ToGlDataType<unsigned int>()
{
  return GL_UNSIGNED_INT;
}
}

template <typename T, BufferType type, BufferUsage usage>
class BufferBase
{
  friend class VertexArray;

public:
  class Value
  {
  public:
    Value() = delete;

    Value(BufferBase<T, type, usage>& buffer, int idx)
      : buffer_(buffer), idx_(idx)
    {
    }

    ~Value() = default;

    Value& operator = (T v)
    {
      buffer_.Modify(idx_, v);
      return *this;
    }

    Value& operator = (const Value& v)
    {
      buffer_.Modify(idx_, v);
      return *this;
    }

    operator T() const
    {
      return buffer_.Get(idx_);
    }

  private:
    BufferBase<T, type, usage>& buffer_;
    int idx_;
  };

  // Empty buffer
  BufferBase()
  {
  }

  explicit BufferBase(int size)
    : size_(size), data_(size, 0)
  {
    Generate();
  }

  explicit BufferBase(std::initializer_list<T> v)
    : size_(v.size()), data_(v)
  {
    Generate();
  }

  ~BufferBase()
  {
    if (generated_)
      glDeleteBuffers(1, &id_);
  }

  // Copy constructors
  BufferBase(const BufferBase<T, type, usage>& rhs)
  {
    // TODO
    std::cout << "buffer copy\n";
  }

  BufferBase& operator = (const BufferBase<T, type, usage>& rhs)
  {
    // TODO
    std::cout << "buffer copy\n";
    return *this;
  }

  // Move constructors
  BufferBase(BufferBase<T, type, usage>&& rhs)
  {
    generated_ = rhs.generated_;
    id_ = rhs.id_;
    need_update_ = rhs.need_update_;
    size_ = rhs.size_;
    data_ = std::move(rhs.data_);

    rhs.generated_ = false;
    rhs.id_ = 0;
    rhs.need_update_ = false;
    rhs.size_ = 0;
  }

  BufferBase& operator = (BufferBase<T, type, usage>&& rhs)
  {
    generated_ = rhs.generated_;
    id_ = rhs.id_;
    need_update_ = rhs.need_update_;
    size_ = rhs.size_;
    data_ = std::move(rhs.data_);

    rhs.generated_ = false;
    rhs.id_ = 0;
    rhs.need_update_ = false;
    rhs.size_ = 0;

    return *this;
  }

  void CopyFrom(const std::vector<T>& data)
  {
    // TODO: if generated already, delete the previous and allocate new one
    if (generated_)
      std::cout << "already generated\n";

    size_ = data.size();
    data_ = data;
    Generate();
  }

  void CopyFrom(const std::vector<T>& data, int offset, int size)
  {
    std::copy(data.begin(), data.begin() + size, data_.begin() + offset);
    need_update_ = true;
  }

  Value operator [] (int idx)
  {
    return Value(*this, idx);
  }

  auto Size() const noexcept
  {
    return size_;
  }

  void Update()
  {
    if (!need_update_)
      return;

    Bind();
    glBufferSubData(internal::ToGlType(type), 0, sizeof(T) * size_, data_.data());
    Unbind();

    need_update_ = false;
  }

private:
  void Bind()
  {
    glBindBuffer(internal::ToGlType(type), id_);
  }

  void Unbind()
  {
    glBindBuffer(internal::ToGlType(type), 0);
  }

  GLenum GlDataType() const
  {
    return internal::ToGlDataType<T>();
  }

  void Generate()
  {
    if (generated_)
      return;

    glGenBuffers(1, &id_);
    Bind();
    glBufferData(internal::ToGlType(type), sizeof(T) * size_, data_.data(), internal::ToGlUsage(usage));
    Unbind();

    need_update_ = false;

    generated_ = true;
  }

  void Modify(int idx, T v)
  {
    data_[idx] = v;
    need_update_ = true;
  }

  T Get(int idx) const
  {
    return data_[idx];
  }

  bool generated_ = false;
  GLuint id_ = 0;

  bool need_update_ = false;

  int size_ = 0;
  std::vector<T> data_;
};

template <typename T>
using Buffer = BufferBase<T, BufferType::ARRAY_BUFFER, BufferUsage::STATIC_DRAW>;

using ElementBuffer = BufferBase<unsigned int, BufferType::ELEMENT_ARRAY_BUFFER, BufferUsage::STATIC_DRAW>;
}

#endif // IPLANNER_OBJECT_BUFFER_H_
