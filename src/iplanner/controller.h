#ifndef IPLANNER_CONTROLLER_H_
#define IPLANNER_CONTROLLER_H_

#include <vector>

#include "iplanner/object/buffer.h"
#include "iplanner/object/vertex_array.h"
#include "iplanner/object/texture.h"
#include "iplanner/shader/program.h"
#include "iplanner/scene/scene.h"
#include "iplanner/types.h"

namespace iplanner
{
class Engine;

class Controller
{
private:
  static const int vertical_gap_ = 10;
  static const int point_half_size_ = 5;
  const float bar_width_ = 2.f;
  const float graph_line_width_ = 1.f;

public:
  Controller() = delete;
  Controller(Engine* engine, int width, int height);
  ~Controller();

  void Resize(int width, int height);
  void Clicked(double x, double y);
  void ClickReleased();
  void MouseMove(double x, double y);

  void SetControllerSize(int rows, int cols);

  void Initialize();
  void Render();

  auto& At(int row, int col)
  {
    return values_[row][col];
  }

  auto At(int row, int col) const
  {
    return values_[row][col];
  }

private:
  // x, y, width, height
  Vector4i CellArea(int row, int col);

  void DrawRect(int x, int y, int width, int height, const Vector3f& color);
  void DrawLine(int x0, int y0, int x1, int y1, const Vector3f& color, float line_width = 1.f);

  void ControlValueWithMousePos(double x, double y);

  Engine* engine_;

  int width_ = 800;
  int height_ = 800;

  int rows_ = 1;
  int cols_ = 1;
  std::vector<std::vector<double>> values_;

  int clicked_row_ = -1;
  int clicked_col_ = -1;

  std::shared_ptr<Program> program_planar_;

  // Rectangle buffer (x y r g b)
  BufferBase<float, BufferType::ARRAY_BUFFER, BufferUsage::DYNAMIC_DRAW> vertex_buffer_;

  // Test
  Buffer<float> test_buffer_{
    100.f, 100.f, 0.f, 0.f, 0.f,
    600.f, 100.f, 1.f, 0.f, 0.f,
    100.f, 400.f, 0.f, 1.f, 0.f,
    600.f, 400.f, 0.f, 0.f, 1.f,
  };

  // Rectangle
  ElementBuffer rect_elements_{ 0, 1, 2, 3 };
  VertexArray rect_vao_;
  VertexArray line_vao_;
};
}

#endif // IPLANNER_CONTROLLER_H_
