#ifndef IPLANNER_ENGINE_H_
#define IPLANNER_ENGINE_H_

#include <array>

#include "iplanner/renderer.h"
#include "iplanner/controller.h"
#include "iplanner/config/config.h"
#include "iplanner/robot/robot_model.h"
#include "iplanner/robot/robot_state.h"
#include "iplanner/scene/scene.h"
#include "iplanner/scene/camera.h"
#include "iplanner/scene/robot_node.h"
#include "iplanner/scene/point_cloud_node.h"
#include "iplanner/scene/human_label_node.h"
#include "iplanner/dataset/dataset_wnp.h"
#include "iplanner/dataset/dataset_utkinect.h"
#include "iplanner/dataset/dataset_occlusion.h"
#include "iplanner/sensor/kinect_v1.h"
#include "iplanner/sensor/kinect_v2.h"
#include "iplanner/sensor/fake_kinect.h"
#include "iplanner/plan/planner.h"

struct GLFWwindow;

namespace iplanner
{
class Engine
{
public:
  enum class Button : uint8_t
  {
    LEFT,
    MIDDLE,
    RIGHT,
    UNKNOWN
  };

  enum class MouseAction : uint8_t
  {
    PRESS,
    RELEASE,
    UNKNOWN
  };

  enum class Key : int16_t
  {
    UNKNOWN = -1,
    SPACE = ' ',
    APOSTROPHE = '\'',
    COMMA = ',',
    MINUS = '-',
    PERIOD = '.',
    SLASH = '/',
    NUM0 = '0',
    NUM1,
    NUM2,
    NUM3,
    NUM4,
    NUM5,
    NUM6,
    NUM7,
    NUM8,
    NUM9,
    SEMICOLON = ';',
    EQUAL = '=',
    A = 'A',
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    M,
    N,
    O,
    P,
    Q,
    R,
    S,
    T,
    U,
    V,
    W,
    X,
    Y,
    Z,
    LEFT_BRACKET = '[',
    BACKSLASH = '\\',
    RIGHT_BRACKET = ']',
    GRAVE_ACCENT = '`',
    WORLD_1 = 161,
    WORLD_2 = 162,
    ESCAPE = 256,
    ENTER = 257,
    TAB = 258,
    BACKSPACE = 259,
    INSERT = 260,
    DELETEKEY = 261,
    RIGHT = 262,
    LEFT = 263,
    DOWN = 264,
    UP = 265,
    PAGE_UP = 266,
    PAGE_DOWN = 267,
    HOME = 268,
    END = 269,
    CAPS_LOCK = 280,
    SCROLL_LOCK = 281,
    NUM_LOCK = 282,
    PRINT_SCREEN = 283,
    PAUSE = 284,
    F1 = 290,
    F2,
    F3,
    F4,
    F5,
    F6,
    F7,
    F8,
    F9,
    F10,
    F11,
    F12,
    F13,
    F14,
    F15,
    F16,
    F17,
    F18,
    F19,
    F20,
    F21,
    F22,
    F23,
    F24,
    F25,
    KP_0 = 320,
    KP_1,
    KP_2,
    KP_3,
    KP_4,
    KP_5,
    KP_6,
    KP_7,
    KP_8,
    KP_9,
    KP_DECIMAL = 330,
    KP_DIVIDE = 331,
    KP_MULTIPLY = 332,
    KP_SUBTRACT = 333,
    KP_ADD = 334,
    KP_ENTER = 335,
    KP_EQUAL = 336,
    LEFT_SHIFT = 340,
    LEFT_CONTROL = 341,
    LEFT_ALT = 342,
    LEFT_SUPER = 343,
    RIGHT_SHIFT = 344,
    RIGHT_CONTROL = 345,
    RIGHT_ALT = 346,
    RIGHT_SUPER = 347,
    MENU = 348,
  };

  enum class KeyAction : uint8_t
  {
    PRESS,
    REPEAT,
    RELEASE,
    UNKNOWN
  };

  enum class ViewMode
  {
    ALL,
    VIEW,
    COLOR_IMAGE,
    COLOR_VIEW,
    DEPTH_IMAGE,
    DEPTH_VIEW,
  };

public:
  Engine();
  ~Engine();

  void Run();

  void MouseButton(GLFWwindow* window, Button button, MouseAction action, int mods);
  void MouseMove(GLFWwindow* window, double x, double y);
  void Keyboard(GLFWwindow* window, Key key, KeyAction action, int mods);
  void Resize(GLFWwindow* window, int width, int height);

  Vector2i GetScreenSize() const;

  void SaveImageSamples();

  auto GetViewMode() const
  {
    return view_mode_;
  }

private:
  void Initialize();
  void InitializeScene();

  void ChooseUtKinect();
  void ChooseWnp();
  void ChooseOcclusion();

  void Update();
  void UpdateScene();

  void UpdateController();

  int width_ = 1280;
  int height_ = 720;

  GLFWwindow* window_renderer_ = 0;
  GLFWwindow* window_controller_ = 0;

  std::array<MouseAction, 3> mouse_button_status_
  {
    MouseAction::RELEASE,
    MouseAction::RELEASE,
    MouseAction::RELEASE
  };

  double mouse_last_x_ = 0.;
  double mouse_last_y_ = 0.;

  // Renderer
  bool redraw_ = true;
  bool animation_ = false;
  bool dataset_changed_ = false;
  double animation_start_time_ = 0.;
  double animation_time_ = 0.;

  bool image_samples_saved_ = false;

  std::unique_ptr<Renderer> renderer_;
  std::shared_ptr<Camera> view_camera_;
  std::shared_ptr<Camera> color_camera_;
  std::shared_ptr<Camera> depth_camera_;
  std::shared_ptr<Scene> scene_;

  ViewMode view_mode_ = ViewMode::ALL;

  // Controller
  bool redraw_controller_ = true;

  std::unique_ptr<Controller> controller_;

  // Config
  Config config_;

  // Planner
  std::shared_ptr<Planner> planner_;

  // Robot
  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<RobotModel> robot_arm_model_;
  std::shared_ptr<RobotState> robot_state_;

  Affine3d head_to_color_camera_transform_;
  Affine3d head_to_depth_camera_transform_;

  // Robot scene node
  std::shared_ptr<RobotNode> robot_node_;

  // Dataset
  std::shared_ptr<UtKinect> dataset_utkinect_;
  std::shared_ptr<Wnp> dataset_wnp_;
  std::shared_ptr<DatasetOcclusion> dataset_occlusion_;
  std::shared_ptr<Dataset> dataset_;

  // Sensor
  std::shared_ptr<RgbdCamera> rgbd_camera_;
  std::shared_ptr<KinectV1> kinect_v1_;
  std::shared_ptr<KinectV2> kinect_v2_;
  std::shared_ptr<FakeKinect> fake_kinect_;

  // Human
  std::shared_ptr<HumanModel> human_model_;

  // Human scene node
  std::shared_ptr<HumanLabelNode> human_label_node_kinect_v2_;
  std::shared_ptr<HumanLabelNode> human_label_node_kinect_v1_;
  std::shared_ptr<HumanLabelNode> human_label_node_fake_kinect_;
  std::shared_ptr<HumanLabelNode> human_label_node_;

  // Pre-allocated point cloud memory
  std::shared_ptr<PointCloud> point_cloud_;

  // Point cloud scene node
  std::shared_ptr<PointCloudNode> point_cloud_node_;
};
}

#endif // IPLANNER_ENGINE_H_
