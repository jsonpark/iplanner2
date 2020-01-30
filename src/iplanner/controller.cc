#include "iplanner/controller.h"

#include <iostream>

#include <glad/glad.h>

#include "iplanner/engine.h"

namespace iplanner
{
Controller::Controller(Engine* engine, int width, int height)
  : engine_(engine),
  width_(width), height_(height),
  vertex_buffer_(20)
{
}

Controller::~Controller() = default;

void Controller::Resize(int width, int height)
{
  width_ = width;
  height_ = height;
}

void Controller::SetControllerSize(int rows, int cols)
{
  rows_ = rows;
  cols_ = cols;

  values_.resize(rows_);
  for (int i = 0; i < rows_; i++)
    values_[i].resize(cols_, 0.);
}

void Controller::Initialize()
{
  glDisable(GL_DEPTH_TEST);
  glClearColor(0.f, 0.f, 0.f, 1.f);

  program_planar_ = std::make_shared<Program>("..\\src\\shader", "planar");

  // Rectangle vao
  rect_vao_.BufferPointer(0, 2, vertex_buffer_, 5, 0);
  rect_vao_.BufferPointer(1, 3, vertex_buffer_, 5, 2);
  rect_vao_.SetDrawElementMode(VertexArray::DrawMode::TRIANGLE_STRIP, rect_elements_);

  line_vao_.BufferPointer(0, 2, vertex_buffer_, 5, 0);
  line_vao_.BufferPointer(1, 3, vertex_buffer_, 5, 2);
  line_vao_.SetDrawElementMode(VertexArray::DrawMode::LINES, rect_elements_, 2);
}

void Controller::Render()
{
  glClear(GL_COLOR_BUFFER_BIT);
  glViewport(0, 0, width_, height_);

  program_planar_->Uniform2i("screen", width_, height_);

  // Draw white background
  for (int i = 0; i < rows_; i++)
  {
    for (int j = 0; j < cols_; j++)
    {
      auto area = CellArea(i, j);

      DrawRect(area(0), area(1), area(2), area(3), Vector3f(1.f, 1.f, 1.f));
    }
  }

  // Vertical separators
  for (int i = 0; i < rows_; i++)
  {
    for (int j = 0; j < cols_; j++)
    {
      auto area = CellArea(i, j);

      DrawLine(area(0) + area(2), area(1), area(0) + area(2), area(1) + area(3), Vector3f(0.f, 0.f, 0.f));
    }
  }

  // Values horizontal bars
  for (int i = 0; i < rows_; i++)
  {
    for (int j = 0; j < cols_; j++)
    {
      auto area = CellArea(i, j);

      double x = area(0) + area(2) / 2.;
      double y = area(1) + values_[i][j] * area(3);

      DrawRect(x - point_half_size_, y - point_half_size_, point_half_size_ * 2, point_half_size_ * 2, Vector3f(0.f, 0.f, 1.f));
      DrawLine(area(0), y, area(0) + area(2), y, Vector3f(1.f, 0.f, 0.f), bar_width_);
    }
  }

  // Graph
  for (int i = 0; i < rows_; i++)
  {
    for (int j = 0; j < cols_ - 1; j++)
    {
      auto area1 = CellArea(i, j);
      auto area2 = CellArea(i, j + 1);

      double x1 = area1(0) + area1(2) / 2.;
      double y1 = area1(1) + values_[i][j] * area1(3);
      double x2 = area2(0) + area2(2) / 2.;
      double y2 = area2(1) + values_[i][j + 1] * area2(3);

      DrawLine(x1, y1, x2, y2, Vector3f(1.f, 1.f, 0.f), graph_line_width_);
    }
  }
}

Vector4i Controller::CellArea(int row, int col)
{
  Vector4i area;

  area(2) = width_ / cols_;
  area(3) = (height_ - (rows_ - 1) * vertical_gap_) / rows_;

  area(0) = area(2) * col;
  area(1) = (area(3) + vertical_gap_) * (rows_ - row - 1);

  return area;
}

void Controller::DrawRect(int x, int y, int width, int height, const Vector3f& color)
{
  constexpr int stride = 5;

  vertex_buffer_[0 * stride + 0] = static_cast<float>(x);
  vertex_buffer_[0 * stride + 1] = static_cast<float>(y);

  vertex_buffer_[1 * stride + 0] = static_cast<float>(x + width);
  vertex_buffer_[1 * stride + 1] = static_cast<float>(y);

  vertex_buffer_[2 * stride + 0] = static_cast<float>(x);
  vertex_buffer_[2 * stride + 1] = static_cast<float>(y + height);

  vertex_buffer_[3 * stride + 0] = static_cast<float>(x + width);
  vertex_buffer_[3 * stride + 1] = static_cast<float>(y + height);

  for (int i = 0; i < 4; i++)
  {
    vertex_buffer_[i * stride + 2] = color(0);
    vertex_buffer_[i * stride + 3] = color(1);
    vertex_buffer_[i * stride + 4] = color(2);
  }

  vertex_buffer_.Update();

  program_planar_->Use();
  rect_vao_.Draw();
}

void Controller::DrawLine(int x0, int y0, int x1, int y1, const Vector3f& color, float line_width)
{
  constexpr int stride = 5;

  glLineWidth(line_width);

  vertex_buffer_[0 * stride + 0] = static_cast<float>(x0);
  vertex_buffer_[0 * stride + 1] = static_cast<float>(y0);

  vertex_buffer_[1 * stride + 0] = static_cast<float>(x1);
  vertex_buffer_[1 * stride + 1] = static_cast<float>(y1);

  for (int i = 0; i < 2; i++)
  {
    vertex_buffer_[i * stride + 2] = color(0);
    vertex_buffer_[i * stride + 3] = color(1);
    vertex_buffer_[i * stride + 4] = color(2);
  }

  vertex_buffer_.Update();

  program_planar_->Use();
  line_vao_.Draw();
}
}
