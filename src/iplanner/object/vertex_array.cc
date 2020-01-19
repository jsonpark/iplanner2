#include "iplanner/object/vertex_array.h"

namespace iplanner
{
VertexArray::VertexArray()
{
  Generate();
}

VertexArray::~VertexArray()
{
  if (generated_)
    glDeleteVertexArrays(1, &id_);
}

void VertexArray::SetDrawElementMode(DrawMode mode, ElementBuffer& buffer)
{
  has_elements_ = true;
  mode_ = mode;
  count_ = buffer.Size();

  Bind();
  buffer.Bind();
  Unbind();
  buffer.Unbind();
}

void VertexArray::SetDrawElementMode(DrawMode mode, ElementBuffer& buffer, int count)
{
  has_elements_ = true;
  mode_ = mode;
  count_ = count;

  Bind();
  buffer.Bind();
  Unbind();
  buffer.Unbind();
}

void VertexArray::SetDrawArrayMode(DrawMode mode, int count)
{
  has_elements_ = false;
  mode_ = mode;
  count_ = count;
}

void VertexArray::SetDrawMode(DrawMode mode)
{
  mode_ = mode;
}

void VertexArray::SetTexture(int id, const std::shared_ptr<Texture>& texture)
{
  if (id >= textures_.size())
    textures_.resize(static_cast<size_t>(id) + 1);

  textures_[id] = texture;
}

void VertexArray::SetVertexCount(int count)
{
  count_ = count;
}

void VertexArray::Draw()
{
  Bind();

  if (!textures_.empty())
  {
    for (int i = 0; i < textures_.size(); i++)
    {
      glActiveTexture(GL_TEXTURE0 + i);

      if (textures_[i] != nullptr)
        textures_[i]->Bind();
    }

    glActiveTexture(GL_TEXTURE0);
  }

  if (has_elements_)
  {
    // Draw elements
    glDrawElements(ToGlDrawMode(mode_), count_, GL_UNSIGNED_INT, 0);
  }

  else
  {
    // Draw arrays
    glDrawArrays(ToGlDrawMode(mode_), 0, count_);
  }

  Unbind();

  if (!textures_.empty())
  {
    for (int i = 0; i < textures_.size(); i++)
    {
      glActiveTexture(GL_TEXTURE0 + i);

      if (textures_[i] != nullptr)
        textures_[i]->Unbind();
    }

    glActiveTexture(GL_TEXTURE0);
  }
}

void VertexArray::Generate()
{
  glGenVertexArrays(1, &id_);
  generated_ = true;
}

void VertexArray::Bind()
{
  glBindVertexArray(id_);
}

void VertexArray::Unbind()
{
  glBindVertexArray(0);
}
}
