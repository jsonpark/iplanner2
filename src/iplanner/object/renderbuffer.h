#ifndef IPLANNER_OBJECT_RENDERBUFFER_H_
#define IPLANNER_OBJECT_RENDERBUFFER_H_

#include <memory>

#include <glad/glad.h>

#include "iplanner/object/texture.h"

namespace iplanner
{
class Renderbuffer
{
public:
  enum class Usage
  {
    COLOR,
    DEPTH,
    UNDEFINED
  };

public:
  Renderbuffer() = delete;

  Renderbuffer(int width, int height, Usage usage = Usage::COLOR);

  ~Renderbuffer();

  auto Id() const noexcept
  {
    return id_;
  }

  void Bind();
  void Unbind();

private:
  void GenerateColorRenderbuffer();
  void GenerateDepthRenderbuffer();

  bool generated_ = false;

  Usage usage_ = Usage::UNDEFINED;

  GLuint id_ = 0;

  int width_ = 0;
  int height_ = 0;
};
}

#endif // IPLANNER_OBJECT_RENDERBUFFER_H_
