#ifndef IPLANNER_OBJECT_TEXTURE_H_
#define IPLANNER_OBJECT_TEXTURE_H_

#include <string>
#include <vector>

#include <glad/glad.h>

namespace iplanner
{
class Texture
{
  friend class VertexArray;

public:
  enum class Usage
  {
    TEXTURE,
    U16_TEXTURE,
    COLOR_FRAMEBUFFER,
    DEPTH_FRAMEBUFFER,
    U32_FRAMEBUFFER,
    DEPTH_STENCIL_FRAMEBUFFER,
    UNDEFINED,
  };

public:
  Texture() = delete;

  // Empty texture
  Texture(int width, int height, Usage usage = Usage::TEXTURE);

  Texture(const std::string& filename);

  Texture(int width, int height, int num_channels, const std::vector<unsigned char>& image);

  ~Texture();

  auto Id() const noexcept
  {
    return id_;
  }

  void Bind();
  void Unbind();
  
  std::vector<unsigned char> GetImage();

  void Update(const void* pixels);

private:
  void GenerateU16Texture();
  void GenerateColorTexture();
  void GenerateDepthTexture();
  void GenerateU32Texture();
  void GenerateDepthStencilTexture();
  void GenerateWithImage(const unsigned char* image);

  bool generated_ = false;

  Usage usage_ = Usage::UNDEFINED;

  std::string filename_;

  GLuint id_ = 0;

  int width_ = 0;
  int height_ = 0;
  int num_channels_ = 0;
};
}

#endif // IPLANNER_OBJECT_TEXTURE_H_
