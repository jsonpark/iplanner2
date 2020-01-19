#include "iplanner/object/renderbuffer.h"

#include <iostream>

namespace iplanner
{
Renderbuffer::Renderbuffer(int width, int height, Usage usage)
  : width_(width), height_(height), usage_(usage)
{
  switch (usage)
  {
  case Usage::COLOR:
    GenerateColorRenderbuffer();
    break;

  case Usage::DEPTH:
    GenerateDepthRenderbuffer();
    break;

  default:
    return;
  }
}

Renderbuffer::~Renderbuffer()
{
  if (generated_)
    glDeleteRenderbuffers(1, &id_);
}

void Renderbuffer::Bind()
{
  glBindRenderbuffer(GL_RENDERBUFFER, id_);
}

void Renderbuffer::Unbind()
{
  glBindRenderbuffer(GL_RENDERBUFFER, 0);
}

void Renderbuffer::GenerateColorRenderbuffer()
{
  glGenRenderbuffers(1, &id_);

  generated_ = true;

  Bind();
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB, width_, height_);
  Unbind();
}

void Renderbuffer::GenerateDepthRenderbuffer()
{
  glGenRenderbuffers(1, &id_);

  generated_ = true;

  Bind();
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width_, height_);
  Unbind();
}
}
