#include "iplanner/object/framebuffer.h"

#include <iostream>

namespace iplanner
{
Framebuffer::Framebuffer(int width, int height)
  : width_(width), height_(height)
{
  Generate();
}

Framebuffer::~Framebuffer()
{
  if (generated_)
    glDeleteFramebuffers(1, &id_);
}

void Framebuffer::Use()
{
  Bind();

  std::vector<GLenum> buffers;
  for (int i = 0; i < texture_colors_.size(); i++)
  {
    if (texture_colors_[i] != nullptr)
      buffers.push_back(GL_COLOR_ATTACHMENT0 + i);
  }

  glDrawBuffers(buffers.size(), buffers.data());
}

void Framebuffer::UseScreen()
{
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glDrawBuffer(GL_BACK);
}

void Framebuffer::Bind()
{
  glBindFramebuffer(GL_FRAMEBUFFER, id_);
}

void Framebuffer::Unbind()
{
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Framebuffer::Generate()
{
  glGenFramebuffers(1, &id_);
  generated_ = true;
}

void Framebuffer::AttachColorTexture(int idx, std::shared_ptr<Texture> texture)
{
  if (idx >= texture_colors_.size())
    texture_colors_.resize(static_cast<size_t>(idx) + 1, nullptr);

  texture_colors_[idx] = texture;

  Bind();

  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + idx, GL_TEXTURE_2D, texture_colors_[idx]->Id(), 0);

  Unbind();
}

void Framebuffer::CreateColorTexture(int idx)
{
  auto texture = std::make_shared<Texture>(width_, height_, Texture::Usage::COLOR_FRAMEBUFFER);

  AttachColorTexture(idx, texture);
}

void Framebuffer::CreateDepthTexture()
{
  Bind();

  texture_depth_ = std::make_shared<Texture>(width_, height_, Texture::Usage::DEPTH_FRAMEBUFFER);

  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texture_depth_->Id(), 0);

  Unbind();
}

void Framebuffer::CreateDepthStencilTexture()
{
  Bind();

  texture_depth_stencil_ = std::make_shared<Texture>(width_, height_, Texture::Usage::DEPTH_STENCIL_FRAMEBUFFER);

  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, texture_depth_stencil_->Id(), 0);

  Unbind();
}

void Framebuffer::CreateDepthStencilRenderbuffer()
{
  Bind();

  depth_renderbuffer_ = std::make_shared<Renderbuffer>(width_, height_, Renderbuffer::Usage::DEPTH);

  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depth_renderbuffer_->Id());

  Unbind();
}
}
