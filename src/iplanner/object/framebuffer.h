#ifndef IPLANNER_OBJECT_FRAMEBUFFER_H_
#define IPLANNER_OBJECT_FRAMEBUFFER_H_

#include <memory>

#include <glad/glad.h>

#include "iplanner/object/texture.h"
#include "iplanner/object/renderbuffer.h"

namespace iplanner
{
class Framebuffer
{
public:
  static void UseScreen();

public:
  Framebuffer() = delete;

  Framebuffer(int width, int height);

  ~Framebuffer();

  void Resize(int width, int height);

  void Bind();
  void Unbind();

  void Use();

  auto GetColorTexture(int idx = 0) const
  {
    return texture_colors_[idx];
  }

  auto GetDepthTexture() const
  {
    return texture_depth_;
  }

  auto GetDepthStencilTexture() const
  {
    return texture_depth_stencil_;
  }

  void AttachColorTexture(int idx, std::shared_ptr<Texture> texture);
  void CreateColorTexture(int idx = 0);
  void CreateDepthTexture();
  void CreateDepthStencilTexture();
  void CreateDepthStencilRenderbuffer();

private:
  void Generate();

  bool generated_ = false;

  GLuint id_ = 0;

  int width_ = 0;
  int height_ = 0;

  std::vector<std::shared_ptr<Texture>> texture_colors_;
  std::shared_ptr<Texture> texture_depth_;
  std::shared_ptr<Texture> texture_depth_stencil_;
  std::shared_ptr<Renderbuffer> depth_renderbuffer_;
};
}

#endif // IPLANNER_OBJECT_FRAMEBUFFER_H_
