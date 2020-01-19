#ifndef IPLANNER_OBJECT_VERTEX_ARRAY_H_
#define IPLANNER_OBJECT_VERTEX_ARRAY_H_

#include <memory>

#include "iplanner/object/buffer.h"
#include "iplanner/object/texture.h"

namespace iplanner
{
class VertexArray
{
public:
  enum class DrawMode
  {
    POINTS,
    LINE_STRIP,
    LINE_LOOP,
    LINES,
    TRIANGLE_STRIP,
    TRIANGLE_FAN,
    TRIANGLES,
    UNDEFINED,
  };

private:
  constexpr GLenum ToGlDrawMode(DrawMode mode)
  {
    switch (mode)
    {
    case DrawMode::POINTS:
      return GL_POINTS;

    case DrawMode::LINE_STRIP:
      return GL_LINE_STRIP;

    case DrawMode::LINE_LOOP:
      return GL_LINE_LOOP;

    case DrawMode::LINES:
      return GL_LINES;

    case DrawMode::TRIANGLE_STRIP:
      return GL_TRIANGLE_STRIP;

    case DrawMode::TRIANGLE_FAN:
      return GL_TRIANGLE_FAN;

    case DrawMode::TRIANGLES:
      return GL_TRIANGLES;

    default:
      return 0;
    }
  }

public:
  VertexArray();
  ~VertexArray();

  // Copy constructors
  VertexArray(const VertexArray& rhs) = delete;

  VertexArray& operator = (const VertexArray& rhs) = delete;

  // Move constructors
  VertexArray(VertexArray&& rhs)
  {
    generated_ = rhs.generated_;
    id_ = rhs.id_;
    mode_ = rhs.mode_;
    has_elements_ = rhs.has_elements_;
    count_ = rhs.count_;
    textures_ = std::move(rhs.textures_);

    rhs.generated_ = false;
    rhs.id_ = 0;
    rhs.mode_ = DrawMode::UNDEFINED;
    rhs.has_elements_ = false;
    rhs.count_ = 0;
  }

  VertexArray& operator = (VertexArray&& rhs)
  {
    generated_ = rhs.generated_;
    id_ = rhs.id_;
    mode_ = rhs.mode_;
    has_elements_ = rhs.has_elements_;
    count_ = rhs.count_;
    textures_ = std::move(rhs.textures_);

    rhs.generated_ = false;
    rhs.id_ = 0;
    rhs.mode_ = DrawMode::UNDEFINED;
    rhs.has_elements_ = false;
    rhs.count_ = 0;

    return *this;
  }

  template<typename T, BufferType type, BufferUsage usage>
  void BufferPointer(int index, int size, BufferBase<T, type, usage>& buffer, int stride = 0, int offset = 0)
  {
    Bind();
    buffer.Bind();

    glVertexAttribPointer(index, size, buffer.GlDataType(), GL_FALSE, sizeof(T) * stride, (void*)(sizeof(T) * offset));
    glEnableVertexAttribArray(index);

    Unbind();
    buffer.Unbind();
  }

  void SetDrawElementMode(DrawMode mode, ElementBuffer& buffer);
  void SetDrawElementMode(DrawMode mode, ElementBuffer& buffer, int count);
  void SetDrawArrayMode(DrawMode mode, int count);
  void SetDrawMode(DrawMode mode);
  void SetVertexCount(int count);

  void SetTexture(int id, const std::shared_ptr<Texture>& texture);

  void Draw();

private:
  void Generate();
  void Bind();
  void Unbind();

  bool generated_ = false;
  GLuint id_ = 0;

  DrawMode mode_ = DrawMode::UNDEFINED;
  bool has_elements_ = false;
  int count_ = 0;

  std::vector<std::shared_ptr<Texture>> textures_;
};
}

#endif // IPLANNER_OBJECT_VERTEX_ARRAY_H_
