#include "iplanner/object/texture.h"

#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

namespace iplanner
{
Texture::Texture(int width, int height, Usage usage)
  : width_(width), height_(height), usage_(usage)
{
  switch (usage_)
  {
  case Usage::TEXTURE:
  case Usage::COLOR_FRAMEBUFFER:
    GenerateColorTexture();
    break;

  case Usage::U16_TEXTURE:
    GenerateU16Texture();
    break;

  case Usage::DEPTH_FRAMEBUFFER:
    GenerateDepthTexture();
    break;

  case Usage::U32_FRAMEBUFFER:
    GenerateU32Texture();
    break;

  case Usage::DEPTH_STENCIL_FRAMEBUFFER:
    GenerateDepthStencilTexture();
    break;

  default:
    return;
  }
}

Texture::Texture(const std::string& filename)
  : filename_(filename)
{
  usage_ = Usage::TEXTURE;

  filename_ = filename;

  stbi_set_flip_vertically_on_load(true);
  unsigned char* image = stbi_load(filename_.c_str(), &width_, &height_, &num_channels_, 0);

  GenerateWithImage(image);

  stbi_image_free(image);
}

Texture::Texture(int width, int height, int num_channels, const std::vector<unsigned char>& image)
  : width_(width), height_(height), num_channels_(num_channels)
{
  usage_ = Usage::TEXTURE;

  GenerateWithImage(image.data());
}

Texture::~Texture()
{
  if (generated_)
    glDeleteTextures(1, &id_);
}

void Texture::GenerateU16Texture()
{
  glGenTextures(1, &id_);

  Bind();
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_R16UI, width_, height_, 0, GL_RED_INTEGER, GL_UNSIGNED_SHORT, 0);

  Unbind();
  generated_ = true;
  num_channels_ = 1;
}

void Texture::GenerateColorTexture()
{
  glGenTextures(1, &id_);

  Bind();
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);

  Unbind();
  generated_ = true;
  num_channels_ = 3;
}

void Texture::GenerateDepthTexture()
{
  glGenTextures(1, &id_);

  Bind();
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width_, height_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

  Unbind();
  generated_ = true;
}

void Texture::GenerateU32Texture()
{
  glGenTextures(1, &id_);

  Bind();

  // Store u32 integer values in red channel,
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R32UI, width_, height_, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, 0);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

  Unbind();
  generated_ = true;
  num_channels_ = 1;
}

void Texture::GenerateDepthStencilTexture()
{
  glGenTextures(1, &id_);

  Bind();
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH24_STENCIL8, width_, height_, 0, GL_DEPTH_STENCIL, GL_UNSIGNED_INT_24_8, 0);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

  Unbind();
  generated_ = true;
}

void Texture::GenerateWithImage(const unsigned char* image)
{
  glGenTextures(1, &id_);
  
  Bind();
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  switch (num_channels_)
  {
  case 1:
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R, width_, height_, 0, GL_R, GL_UNSIGNED_BYTE, image);

  case 3:
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
    break;

  case 4:
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width_, height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);
    break;
  }

  glGenerateMipmap(GL_TEXTURE_2D);

  Unbind();
  generated_ = true;
}

void Texture::Bind()
{
  glBindTexture(GL_TEXTURE_2D, id_);
}

void Texture::Unbind()
{
  glBindTexture(GL_TEXTURE_2D, 0);
}

std::vector<unsigned char> Texture::GetImage()
{
  std::vector<unsigned char> image;

  switch (usage_)
  {
  case Usage::TEXTURE:
  case Usage::COLOR_FRAMEBUFFER:
    Bind();

    image.resize(width_ * height_ * 3);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, image.data());

    Unbind();

    break;

    // TODO: get image buffer for other types of textures

  default:
    break;
  }

  return image;
}

void Texture::Update(const void* pixels)
{
  GLenum gl_format = 0;
  GLenum gl_type = 0;

  switch (usage_)
  {
  case Usage::TEXTURE:
  case Usage::COLOR_FRAMEBUFFER:
    switch (num_channels_)
    {
    case 1:
      gl_format = GL_R;
      break;

    case 3:
      gl_format = GL_RGB;
      break;

    case 4:
      gl_format = GL_RGBA;
      break;
    }
    gl_type = GL_UNSIGNED_BYTE;
    break;

  case Usage::U16_TEXTURE:
    gl_format = GL_RED_INTEGER;
    gl_type = GL_UNSIGNED_SHORT;
    break;

  case Usage::DEPTH_FRAMEBUFFER:
    gl_format = GL_DEPTH_COMPONENT;
    gl_type = GL_FLOAT;
    break;

  case Usage::U32_FRAMEBUFFER:
    gl_format = GL_RED_INTEGER;
    gl_type = GL_UNSIGNED_INT;
    break;

  case Usage::DEPTH_STENCIL_FRAMEBUFFER:
    gl_format = GL_DEPTH_STENCIL;
    gl_type = GL_UNSIGNED_INT_24_8;
    break;

  default:
    return;
  }

  Bind();

  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_, gl_format, gl_type, pixels);

  Unbind();
}

void Texture::Resize(int width, int height)
{
  width_ = width;
  height_ = height;

  GLenum gl_internal_format = 0;
  GLenum gl_format = 0;
  GLenum gl_type = 0;

  switch (usage_)
  {
  case Usage::TEXTURE:
  case Usage::COLOR_FRAMEBUFFER:
    switch (num_channels_)
    {
    case 1:
      gl_format = GL_R;
      break;

    case 3:
      gl_format = GL_RGB;
      break;

    case 4:
      gl_format = GL_RGBA;
      break;
    }
    gl_internal_format = gl_format;
    gl_type = GL_UNSIGNED_BYTE;
    break;

  case Usage::U16_TEXTURE:
    gl_internal_format = GL_R16UI;
    gl_format = GL_RED_INTEGER;
    gl_type = GL_UNSIGNED_SHORT;
    break;

  case Usage::DEPTH_FRAMEBUFFER:
    gl_internal_format = GL_DEPTH_COMPONENT;
    gl_format = GL_DEPTH_COMPONENT;
    gl_type = GL_FLOAT;
    break;

  case Usage::U32_FRAMEBUFFER:
    gl_internal_format = GL_R32UI;
    gl_format = GL_RED_INTEGER;
    gl_type = GL_UNSIGNED_INT;
    break;

  case Usage::DEPTH_STENCIL_FRAMEBUFFER:
    gl_internal_format = GL_DEPTH_STENCIL;
    gl_format = GL_DEPTH_STENCIL;
    gl_type = GL_UNSIGNED_INT_24_8;
    break;

  default:
    return;
  }

  Bind();

  glTexImage2D(GL_TEXTURE_2D, 0, gl_internal_format, width_, height_, 0, gl_format, gl_type, 0);

  glGenerateMipmap(GL_TEXTURE_2D);

  Unbind();
}
}
