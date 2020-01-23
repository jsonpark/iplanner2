#include "iplanner/engine.h"

#include <iostream>
#include <thread>
#include <stdexcept>

#include <glad/glad.h>
#include <glfw/glfw3.h>

#include "iplanner/robot/robot_model_loader.h"
#include "iplanner/scene/ground_node.h"

namespace iplanner
{
namespace
{
void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
  Engine::Button engine_button = Engine::Button::UNKNOWN;

  switch (button)
  {
  case GLFW_MOUSE_BUTTON_LEFT:
    engine_button = Engine::Button::LEFT;
    break;

  case GLFW_MOUSE_BUTTON_MIDDLE:
    engine_button = Engine::Button::MIDDLE;
    break;

  case GLFW_MOUSE_BUTTON_RIGHT:
    engine_button = Engine::Button::RIGHT;
    break;
  }

  Engine::MouseAction engine_action = Engine::MouseAction::UNKNOWN;

  switch (action)
  {
  case GLFW_PRESS:
    engine_action = Engine::MouseAction::PRESS;
    break;

  case GLFW_RELEASE:
    engine_action = Engine::MouseAction::RELEASE;
    break;
  }

  auto engine = static_cast<Engine*>(glfwGetWindowUserPointer(window));
  engine->MouseButton(engine_button, engine_action, mods);
}

void CursorPosCallback(GLFWwindow* window, double xpos, double ypos)
{
  auto engine = static_cast<Engine*>(glfwGetWindowUserPointer(window));
  engine->MouseMove(xpos, ypos);
}

void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
  Engine::Key engine_key = static_cast<Engine::Key>(key);

  Engine::KeyAction engine_action = Engine::KeyAction::UNKNOWN;

  switch (action)
  {
  case GLFW_PRESS:
    engine_action = Engine::KeyAction::PRESS;
    break;

  case GLFW_REPEAT:
    engine_action = Engine::KeyAction::REPEAT;
    break;

  case GLFW_RELEASE:
    engine_action = Engine::KeyAction::RELEASE;
    break;
  }

  auto engine = static_cast<Engine*>(glfwGetWindowUserPointer(window));
  engine->Keyboard(engine_key, engine_action, mods);
}

void FramebufferSizeCallback(GLFWwindow* window, int width, int height)
{
  auto engine = static_cast<Engine*>(glfwGetWindowUserPointer(window));
  engine->Resize(width, height);
}
}

Engine::Engine()
{
  config_.Load("..\\config\\config.json");
}

Engine::~Engine()
{
}

void Engine::MouseButton(Button button, MouseAction action, int mods)
{
  if (button == Button::LEFT)
    mouse_button_status_[0] = action;

  else if (button == Button::MIDDLE)
    mouse_button_status_[1] = action;

  else if (button == Button::RIGHT)
    mouse_button_status_[2] = action;

  glfwGetCursorPos(window_, &mouse_last_x_, &mouse_last_y_);

  redraw_ = true;
}

void Engine::MouseMove(double x, double y)
{
  auto dx = static_cast<float>(x - mouse_last_x_);
  auto dy = static_cast<float>(y - mouse_last_y_);

  if (mouse_button_status_[1] != MouseAction::PRESS)
  {
    // Zoom
    if (mouse_button_status_[0] == MouseAction::PRESS &&
      mouse_button_status_[2] == MouseAction::PRESS)
    {
      view_camera_->Zoom(dy);
    }

    else if (mouse_button_status_[0] == MouseAction::PRESS)
    {
      view_camera_->Rotate(dx, dy);
    }

    else if (mouse_button_status_[2] == MouseAction::PRESS)
    {
      view_camera_->Translate(dx, dy);
    }
  }

  mouse_last_x_ = x;
  mouse_last_y_ = y;

  redraw_ = true;
}

void Engine::Keyboard(Key key, KeyAction action, int mods)
{
  if (action == KeyAction::PRESS)
  {
    switch (key)
    {
    case Key::SPACE:
      if (!animation_)
      {
        animation_ = true;
        animation_start_time_ = glfwGetTime() - animation_time_;
      }
      else
      {
        animation_ = false;
      }
      break;

    case Key::NUM1:
      view_mode_ = ViewMode::ALL;
      break;

    case Key::NUM2:
      view_mode_ = ViewMode::VIEW;
      break;

    case Key::NUM3:
      view_mode_ = ViewMode::COLOR_IMAGE;
      break;

    case Key::NUM4:
      view_mode_ = ViewMode::COLOR_VIEW;
      break;

    case Key::NUM5:
      view_mode_ = ViewMode::DEPTH_IMAGE;
      break;

    case Key::NUM6:
      view_mode_ = ViewMode::DEPTH_VIEW;
      break;

    case Key::UP:
      animation_time_ = 0.;
      animation_start_time_ = glfwGetTime();
      dataset_->PreviousSequence();
      break;

    case Key::DOWN:
      animation_time_ = 0.;
      animation_start_time_ = glfwGetTime();
      dataset_->NextSequence();
      break;

    case Key::R:
      animation_time_ = 0.;
      animation_start_time_ = glfwGetTime();
      break;

    case Key::S:
      SaveImageSamples();
      break;
    }
  }
}

void Engine::Resize(int width, int height)
{
  width_ = width;
  height_ = height;

  redraw_ = true;
}

void Engine::SaveImageSamples()
{
  if (image_samples_saved_)
  {
    std::cout << "Image samples already saved" << std::endl;
    return;
  }

  std::cout << "save directory: " << config_.GetImageSampleSaveDirectory() << std::endl;
  for (const auto& image_sample : config_.GetImageSamples())
  {
    std::cout << image_sample.dataset << ' '
      << image_sample.sequence << ' ' << image_sample.index << std::endl;

    if (image_sample.dataset == "wnp")
    {
      dataset_wnp_->SelectSequenceFrame(image_sample.sequence, image_sample.index);

      Update();

      renderer_->Render();

      std::string filename = image_sample.dataset + '-' + image_sample.sequence + '-' + image_sample.index;
      std::replace(filename.begin(), filename.end(), '\\', '-');
      std::cout << "Saving to " << filename << "*.*" << std::endl;
      renderer_->SaveImages(config_.GetImageSampleSaveDirectory(), filename);
      std::cout << "Finished saving images" << std::endl;
    }
  }

  image_samples_saved_ = true;
}

void Engine::Run()
{
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window_ = glfwCreateWindow(width_, height_, "IplannerEngine", NULL, NULL);
  if (window_ == NULL)
  {
    glfwTerminate();
    throw std::runtime_error("Engine: Failed to create GLFW window.");
  }

  glfwMakeContextCurrent(window_);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    throw std::runtime_error("Engine: Failed to initialize GLAD.");


  // Callbacks
  glfwSetWindowUserPointer(window_, this);
  glfwSetMouseButtonCallback(window_, MouseButtonCallback);
  glfwSetCursorPosCallback(window_, CursorPosCallback);
  glfwSetKeyCallback(window_, KeyCallback);
  glfwSetFramebufferSizeCallback(window_, FramebufferSizeCallback);

  // Renderer
  Initialize();

  while (!glfwWindowShouldClose(window_))
  {
    using namespace std::chrono_literals;

    glfwPollEvents();

    Update();

    if (redraw_)
    {
      renderer_->Render();

      glfwSwapBuffers(window_);

      redraw_ = false;
    }

    // Let CPU be idle for a short time window
    std::this_thread::sleep_for(1ms);
  }

  glfwTerminate();
}

void Engine::Initialize()
{
  glfwWindowHint(GLFW_SAMPLES, 4);
  glEnable(GL_MULTISAMPLE);

  renderer_ = std::make_unique<Renderer>(this);
  renderer_->Initialize();

  constexpr double pi = 3.1415926535897932384626433832795;
  view_camera_ = std::make_shared<Camera>();
  view_camera_->SetAspect(1280.f / 720.f);
  view_camera_->SetFovy(60.f / 180.f * pi);
  view_camera_->SetFar(10.f);

  // Kinect V2 has color image resolution of 1920 x 1080 pixels and a fov of 84.1 x 53.8 resulting in an average of about 22 x 20 pixels per degree.
  // https://smeenk.com/kinect-field-of-view-comparison/
  color_camera_ = std::make_shared<Camera>();
  //color_camera_->SetAspect(std::atan(84.1f / 180.f * pi) / std::atan(53.8f / 180.f * pi));
  color_camera_->SetAspect(1920.f / 1080.f);
  color_camera_->SetFovy(53.8f / 180.f * pi);
  color_camera_->SetFar(10.f);

  // Kinect V2 has a depth image resolution of 512 x 424 pixels with a fov of 70.6 x 60 degrees resulting in an average of about 7 x 7 pixels per degree
  depth_camera_ = std::make_shared<Camera>();
  depth_camera_->SetAspect(std::atan(70.6f / 180.f * pi) / std::atan(60.f / 180.f * pi));
  //depth_camera_->SetAspect(512.f / 424.f);
  depth_camera_->SetFovy(60.f / 180.f * pi);
  depth_camera_->SetFar(10.f);

  scene_ = std::make_shared<Scene>();
  scene_->SetCamera("view", view_camera_);
  scene_->SetCamera("color", color_camera_);
  scene_->SetCamera("depth", depth_camera_);

  renderer_->SetScene(scene_);


  // Light
  Light light;
  light.type = Light::Type::Directional;
  light.position = Vector3f(0.f, 0.f, 1.f);
  light.ambient = Vector3f(0.1f, 0.1f, 0.1f);
  light.diffuse = Vector3f(0.1f, 0.1f, 0.1f);
  light.specular = Vector3f(0.2f, 0.2f, 0.2f);

  scene_->AddLight(light);

  light.position = Vector3f(-1.f, -1.f, 0.f);
  scene_->AddLight(light);

  light.position = Vector3f(-1.f, 1.f, 0.f);
  scene_->AddLight(light);

  light.position = Vector3f(1.f, -1.f, 0.f);
  scene_->AddLight(light);

  light.position = Vector3f(1.f, 1.f, 0.f);
  scene_->AddLight(light);


  // Robot model
  RobotModelLoader robot_model_loader;
  robot_model_loader.SetPackageDirectory("..\\..\\fetch_ros");
  robot_model_ = robot_model_loader.Load("fetch_description\\robots\\fetch.urdf");


  /* Full robot model
  Robot [fetch] links:
   0: base_link
   1: r_wheel_link
   2: l_wheel_link
   3: torso_lift_link
   4: head_pan_link
   5: head_tilt_link
   6: shoulder_pan_link
   7: shoulder_lift_link
   8: upperarm_roll_link
   9: elbow_flex_link
  10: forearm_roll_link
  11: wrist_flex_link
  12: wrist_roll_link
  13: gripper_link
  14: r_gripper_finger_link
  15: l_gripper_finger_link
  16: bellows_link
  17: bellows_link2
  18: estop_link
  19: laser_link
  20: torso_fixed_link
  21: head_camera_link
  22: head_camera_rgb_frame
  23: head_camera_rgb_optical_frame
  24: head_camera_depth_frame
  25: head_camera_depth_optical_frame

  Robot [fetch] joints:
   0: r_wheel_joint
   1: l_wheel_joint
   2: torso_lift_joint
   3: head_pan_joint
   4: head_tilt_joint
   5: shoulder_pan_joint
   6: shoulder_lift_joint
   7: upperarm_roll_joint
   8: elbow_flex_joint
   9: forearm_roll_joint
  10: wrist_flex_joint
  11: wrist_roll_joint
  12: gripper_axis
  13: r_gripper_finger_joint
  14: l_gripper_finger_joint
  15: bellows_joint
  16: bellows_joint2
  17: estop_joint
  18: laser_joint
  19: torso_fixed_joint
  20: head_camera_joint
  21: head_camera_rgb_joint
  22: head_camera_rgb_optical_joint
  23: head_camera_depth_joint
  24: head_camera_depth_optical_joint
  */

  /* Robot planning model:
  Robot [] links:
   0: base_link
   1: torso_lift_link
   2: head_pan_link
   3: head_tilt_link
   4: shoulder_pan_link
   5: shoulder_lift_link
   6: upperarm_roll_link
   7: elbow_flex_link
   8: forearm_roll_link
   9: wrist_flex_link
  10: wrist_roll_link
  11: r_gripper_finger_link
  12: l_gripper_finger_link

  Robot [] joints:
   0: torso_lift_joint
   1: head_pan_joint
   2: head_tilt_joint
   3: shoulder_pan_joint
   4: shoulder_lift_joint
   5: upperarm_roll_joint
   6: elbow_flex_joint
   7: forearm_roll_joint
   8: wrist_flex_joint
   9: wrist_roll_joint
  10: r_gripper_finger_joint
  11: l_gripper_finger_joint
  */

  std::vector<std::string> fixed_joints = {
    "r_wheel_joint",
    "l_wheel_joint",
    "bellows_joint",
  };
  std::vector<double> joint_values = {
    0.0,
    0.0,
    0.2,
  };

  // Get head-to-camera transforms
  // head_tilt_link -> head_camera_rgb_optical_frame
  // head_tilt_link -> head_camera_depth_optical_frame
  RobotState robot_head_state{ robot_model_ };
  robot_head_state.ForwardKinematics();
  Affine3d head_transform = robot_head_state.GetTransform(5);
  Affine3d color_camera_transform = robot_head_state.GetTransform(23);
  Affine3d depth_camera_transform = robot_head_state.GetTransform(25);
  head_to_color_camera_transform_ = head_transform.inverse() * color_camera_transform;
  head_to_depth_camera_transform_ = head_transform.inverse() * depth_camera_transform;

  robot_arm_model_ = std::make_shared<RobotModel>(robot_model_->FixJoints(fixed_joints, joint_values));

  robot_state_ = std::make_shared<RobotState>(robot_arm_model_);

  //robot_state_->JointPosition(0) = 0.193075;
  robot_state_->JointPosition(3) = 0.345;
  robot_state_->JointPosition(4) = -0.2338021;
  robot_state_->JointPosition(5) = 0.21;
  robot_state_->JointPosition(6) = -0.331486;
  robot_state_->JointPosition(7) = -0.455;
  robot_state_->JointPosition(8) = -0.231486;
  robot_state_->JointPosition(9) = 1.31874;
  robot_state_->JointPosition(10) = 0.025;
  robot_state_->JointPosition(11) = 0.025;

  // Dataset
  dataset_wnp_ = std::make_shared<Wnp>(config_.GetDatasetDirectory("wnp"));
  dataset_utkinect_ = std::make_shared<UtKinect>(config_.GetDatasetDirectory("utkinect"));

  dataset_ = dataset_wnp_;

  point_cloud_ = std::make_shared<PointCloud>();

  renderer_->CreateEmptyTexture("data_color", 1920, 1080, Texture::Usage::TEXTURE);
  renderer_->CreateEmptyTexture("data_depth", 424, 512, Texture::Usage::U16_TEXTURE); // data has column-major depth texture

  // Human
  human_model_ = dataset_wnp_->GetHumanModel();

  // Scene
  InitializeScene();
}

void Engine::InitializeScene()
{
  // Robot scene
  robot_node_ = RobotNode::Create(robot_arm_model_);
  auto root = scene_->GetRootNode();
  SceneNode::Connect(root, robot_node_);

  auto ground = std::make_shared<GroundNode>();
  SceneNode::Connect(root, ground);


  // Point cloud
  point_cloud_node_ = std::make_shared<PointCloudNode>();
  point_cloud_node_->SetPointCloud(point_cloud_);
  SceneNode::Connect(root, point_cloud_node_);


  // Human label scene
  human_label_node_ = std::make_shared<HumanLabelNode>();
  human_label_node_->AddEdge(0, 1);
  human_label_node_->AddEdge(1, 20);
  human_label_node_->AddEdge(20, 2);
  human_label_node_->AddEdge(2, 3);
  human_label_node_->AddEdge(20, 4);
  human_label_node_->AddEdge(4, 5);
  human_label_node_->AddEdge(5, 6);
  human_label_node_->AddEdge(6, 7);
  human_label_node_->AddEdge(7, 21);
  human_label_node_->AddEdge(6, 22);
  human_label_node_->AddEdge(20, 8);
  human_label_node_->AddEdge(8, 9);
  human_label_node_->AddEdge(9, 10);
  human_label_node_->AddEdge(10, 11);
  human_label_node_->AddEdge(11, 23);
  human_label_node_->AddEdge(10, 24);
  human_label_node_->AddEdge(0, 12);
  human_label_node_->AddEdge(12, 13);
  human_label_node_->AddEdge(13, 14);
  human_label_node_->AddEdge(14, 15);
  human_label_node_->AddEdge(0, 16);
  human_label_node_->AddEdge(16, 17);
  human_label_node_->AddEdge(17, 18);
  human_label_node_->AddEdge(18, 19);

  // Additional supports near spine
  human_label_node_->AddEdge(1, 4);
  human_label_node_->AddEdge(1, 8);
  human_label_node_->AddEdge(1, 12);
  human_label_node_->AddEdge(1, 16);

  SceneNode::Connect(root, human_label_node_);
}

void Engine::Update()
{
  auto& state = *robot_state_;

  double t = glfwGetTime() - animation_start_time_;

  if (animation_)
  {
    animation_time_ = t;
    auto frame_number = static_cast<int>(animation_time_ * dataset_->FrameRate());
    dataset_->SelectFrame(frame_number);
  }

  // Update point cloud
  kinect_.FeedFrame(dataset_->GetRgbImage(), dataset_->GetDepthImage());
  kinect_.GeneratePointCloud();
  kinect_.GetPointCloud(point_cloud_);
  point_cloud_node_->UpdateBuffer();

  // Update color/depth images from Kinect sensor
  renderer_->UpdateTexture("data_color", kinect_.GetColorBuffer());
  renderer_->UpdateTexture("data_depth", kinect_.GetDepthBuffer());

  // Update human model
  human_label_node_->SetLabel(dataset_wnp_->GetHumanLabel());

  // sin function motion
  double v = (std::sin(5. * animation_time_) + 1.) / 2. * 0.2;
  state.JointPosition(4) = -0.3 + v;

  state.ForwardKinematics();

  static constexpr double pi = 3.1415926535897932384626433832795;
  Affine3d camera_transform = Affine3d::Identity();
  camera_transform.rotate(AngleAxisd(pi / 2., Vector3d(0., 1., 0.)));
  //camera_transform.translate(Vector3d(0., 1., 0.));
  color_camera_->SetTransform(state.GetTransform(3) * head_to_color_camera_transform_ * camera_transform);
  depth_camera_->SetTransform(state.GetTransform(3) * head_to_depth_camera_transform_ * camera_transform);

  Affine3d point_cloud_transform = Affine3d::Identity();
  point_cloud_transform.rotate(AngleAxisd(pi / 2., Vector3d(0., 0., 1.)));
  point_cloud_transform.rotate(AngleAxisd(pi / 2., Vector3d(0., 1., 0.)));
  //point_cloud_transform.pretranslate(Vector3d(0., -1., 0.));
  point_cloud_node_->SetTransform(state.GetTransform(3) * head_to_color_camera_transform_ * point_cloud_transform);

  human_label_node_->SetTransform(state.GetTransform(3) * head_to_color_camera_transform_ * point_cloud_transform);

  UpdateScene();
}

void Engine::UpdateScene()
{
  auto& state = *robot_state_;

  for (int i = 0; i < state.NumJoints(); i++)
    robot_node_->SetJointValue(state.JointName(i), state.JointPosition(i));

  redraw_ = true;
}

Vector2i Engine::GetScreenSize() const
{
  return Vector2i(width_, height_);
}
}
