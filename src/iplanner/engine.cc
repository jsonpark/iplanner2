#include "iplanner/engine.h"

#include <iostream>
#include <thread>
#include <stdexcept>
#include <chrono>
#include <iomanip>
#include <filesystem>

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
  engine->MouseButton(window, engine_button, engine_action, mods);
}

void CursorPosCallback(GLFWwindow* window, double xpos, double ypos)
{
  auto engine = static_cast<Engine*>(glfwGetWindowUserPointer(window));
  engine->MouseMove(window, xpos, ypos);
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
  engine->Keyboard(window, engine_key, engine_action, mods);
}

void FramebufferSizeCallback(GLFWwindow* window, int width, int height)
{
  auto engine = static_cast<Engine*>(glfwGetWindowUserPointer(window));
  engine->Resize(window, width, height);
}
}

Engine::Engine()
{
  config_.Load("..\\config\\config.json");
}

Engine::~Engine()
{
}

void Engine::MouseButton(GLFWwindow* window, Button button, MouseAction action, int mods)
{
  // Renderer
  if (window == window_renderer_)
  {
    if (button == Button::LEFT)
      mouse_button_status_[0] = action;

    else if (button == Button::MIDDLE)
      mouse_button_status_[1] = action;

    else if (button == Button::RIGHT)
      mouse_button_status_[2] = action;

    glfwGetCursorPos(window, &mouse_last_x_, &mouse_last_y_);

    redraw_ = true;
  }

  // Controller
  else if (window == window_controller_)
  {
    double x, y;
    glfwGetCursorPos(window, &x, &y);

    if (button == Button::LEFT && action == MouseAction::PRESS)
      controller_->Clicked(x, y);

    else if (button == Button::LEFT && action == MouseAction::RELEASE)
      controller_->ClickReleased();

    redraw_controller_ = true;
  }
}

void Engine::MouseMove(GLFWwindow* window, double x, double y)
{
  // Renderer
  if (window == window_renderer_)
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

  // Controller
  else if (window == window_controller_)
  {
    controller_->MouseMove(x, y);

    redraw_controller_ = true;
  }
}

void Engine::Keyboard(GLFWwindow* window, Key key, KeyAction action, int mods)
{
  // Renderer
  if (window == window_renderer_)
  {
    if (action == KeyAction::PRESS)
    {
      switch (key)
      {
        // Play/pause video
      case Key::SPACE:
        if (!animation_)
        {
          animation_ = true;
          animation_start_time_ = glfwGetTime() - animation_time_;
        }
        else
          animation_ = false;
        break;

        // Hide/show future nodes
      case Key::F:
        for (int i = 0; i < robot_future_nodes_.size(); i++)
        {
          if (robot_future_nodes_[i]->IsShown())
            robot_future_nodes_[i]->Hide();
          else
            robot_future_nodes_[i]->Show();
        }
        break;

        // Overlay color
      case Key::C:
        renderer_->SwitchOverlayColorRed();
        break;

      case Key::B:
        renderer_->SwitchOverlayColorBlue();
        break;

        // Number 1-6 selects view mode
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

        // Keyboard up/down changes sequence
      case Key::UP:
        SaveCurrentTrajectory();
        animation_time_ = 0.;
        animation_start_time_ = glfwGetTime();
        dataset_->PreviousSequence();
        sequence_changed_ = true;
        break;

      case Key::DOWN:
        SaveCurrentTrajectory();
        animation_time_ = 0.;
        animation_start_time_ = glfwGetTime();
        dataset_->NextSequence();
        sequence_changed_ = true;
        break;

        // Keyboard left/right to move video by 1 sec
      case Key::LEFT:
        if (animation_time_ < 1.)
        {
          animation_time_ = 0.;
          animation_start_time_ = glfwGetTime();
        }
        else
        {
          animation_time_ -= 1.;
          animation_start_time_ += 1.;
        }
        break;

      case Key::RIGHT:
        animation_time_ += 1.;
        animation_start_time_ -= 1.;
        break;

        // Replay
      case Key::R:
        animation_time_ = 0.;
        animation_start_time_ = glfwGetTime();
        animation_ = false;
        break;

        // Save sample images
      case Key::S:
        SaveImageSamples();
        break;

        // Screen shot
      case Key::F8:
        SaveScreenShot();
        break;

        // Save video
      case Key::V:
        SaveVideo();
        break;

        // Numpad (KP_x) sets a dataset
      case Key::KP_1:
        SaveCurrentTrajectory();
        ChooseUtKinect();
        break;

      case Key::KP_2:
        SaveCurrentTrajectory();
        ChooseWnp();
        break;

      case Key::KP_3:
        SaveCurrentTrajectory();
        ChooseOcclusion();
        break;

        // Page up/down to sample images
      case Key::PAGE_UP:
        SaveCurrentTrajectory();
        ChoosePreviousSampleSequence();
        break;

      case Key::PAGE_DOWN:
        SaveCurrentTrajectory();
        ChooseNextSampleSequence();
        break;

      case Key::HOME:
        SaveCurrentTrajectory();
        ChooseCurrentSampleSequence();
        break;
      }
    }
  }
}

void Engine::ChooseCurrentSampleSequence()
{
  const auto& sample_sequence = sample_sequences_[sample_sequence_index_];

  if (sample_sequence.dataset == "utkinect")
    ChooseUtKinect();

  else if (sample_sequence.dataset == "wnp")
    ChooseWnp();

  else if (sample_sequence.dataset == "occlusion")
    ChooseOcclusion();

  if (sample_sequence_index_ == 4)
    sample_sequence_index_ = sample_sequence_index_;

  dataset_->SelectSequence(sample_sequence.sequence);
  sequence_changed_ = true;
}

void Engine::ChoosePreviousSampleSequence()
{
  if (sample_sequence_index_ > 0)
    sample_sequence_index_--;

  ChooseCurrentSampleSequence();
}

void Engine::ChooseNextSampleSequence()
{
  if (sample_sequence_index_ < sample_sequences_.size() - 1)
    sample_sequence_index_++;

  ChooseCurrentSampleSequence();
}

void Engine::ChooseUtKinect()
{
  if (dataset_ != dataset_utkinect_)
  {
    dataset_ = dataset_utkinect_;
    rgbd_camera_ = kinect_v1_;

    human_label_node_kinect_v1_->Show();
    human_label_node_kinect_v2_->Hide();
    human_label_node_fake_kinect_->Hide();
    human_label_node_ = human_label_node_kinect_v1_;

    animation_time_ = 0.;
    animation_start_time_ = glfwGetTime();
    animation_ = false;

    dataset_changed_ = true;
    sequence_changed_ = true;
  }
}

void Engine::ChooseWnp()
{
  if (dataset_ != dataset_wnp_)
  {
    dataset_ = dataset_wnp_;
    rgbd_camera_ = kinect_v2_;

    human_label_node_kinect_v1_->Hide();
    human_label_node_kinect_v2_->Show();
    human_label_node_fake_kinect_->Hide();
    human_label_node_ = human_label_node_kinect_v2_;

    animation_time_ = 0.;
    animation_start_time_ = glfwGetTime();
    animation_ = false;

    dataset_changed_ = true;
    sequence_changed_ = true;
  }
}

void Engine::ChooseOcclusion()
{
  if (dataset_ != dataset_occlusion_)
  {
    dataset_ = dataset_occlusion_;
    rgbd_camera_ = fake_kinect_;

    human_label_node_kinect_v1_->Hide();
    human_label_node_kinect_v2_->Hide();
    human_label_node_fake_kinect_->Show();
    human_label_node_ = human_label_node_fake_kinect_;

    animation_time_ = 0.;
    animation_start_time_ = glfwGetTime();
    animation_ = false;

    dataset_changed_ = true;
    sequence_changed_ = true;
  }
}

void Engine::Resize(GLFWwindow* window, int width, int height)
{
  if (window == window_renderer_)
  {
    width_ = width;
    height_ = height;

    redraw_ = true;
  }

  else if (window == window_controller_)
  {
    controller_->Resize(width, height);

    redraw_controller_ = true;
  }
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

    if (image_sample.dataset == "utkinect")
    {
      ChooseUtKinect();

      dataset_utkinect_->SelectSequenceFrame(image_sample.sequence, image_sample.index);
      sequence_changed_ = true;

      Update();

      renderer_->Render();

      std::string filename = image_sample.dataset + '-' + image_sample.sequence + '-' + image_sample.index;
      std::replace(filename.begin(), filename.end(), '\\', '-');
      std::cout << "Saving to " << filename << "*.* ... ";
      renderer_->SaveImages(config_.GetImageSampleSaveDirectory(), filename);
      std::cout << "Finished!" << std::endl;
    }

    else if (image_sample.dataset == "wnp")
    {
      ChooseWnp();

      dataset_wnp_->SelectSequenceFrame(image_sample.sequence, image_sample.index);
      sequence_changed_ = true;

      Update();

      renderer_->Render();

      std::string filename = image_sample.dataset + '-' + image_sample.sequence + '-' + image_sample.index;
      std::replace(filename.begin(), filename.end(), '\\', '-');
      std::cout << "Saving to " << filename << "*.* ... ";
      renderer_->SaveImages(config_.GetImageSampleSaveDirectory(), filename);
      std::cout << "Finished!" << std::endl;
    }

    else if (image_sample.dataset == "occlusion")
    {
      ChooseOcclusion();

      dataset_occlusion_->SelectSequenceFrame(image_sample.sequence, image_sample.index);
      sequence_changed_ = true;

      Update();

      renderer_->Render();

      std::string filename = image_sample.dataset + '-' + image_sample.sequence + '-' + image_sample.index;
      std::replace(filename.begin(), filename.end(), '\\', '-');
      std::cout << "Saving to " << filename << "*.* ... ";
      renderer_->SaveImages(config_.GetImageSampleSaveDirectory(), filename);
      std::cout << "Finished!" << std::endl;
    }
  }

  image_samples_saved_ = true;
}

void Engine::SaveScreenShot()
{
  namespace fs = std::filesystem;

  std::string sequence_name = dataset_->GetCurrentSequenceName();
  std::replace(sequence_name.begin(), sequence_name.end(), '\\', '-');

  // TODO: linux directory character
  std::string directory = config_.GetImageSampleSaveDirectory();

  std::cout << "Screen shot save directory: " << directory << std::endl;

  auto ZeroPeddingIndex = [](int digits, int index) {
    std::string x(digits, '0');

    for (int i = 0; i < digits; i++)
    {
      x[digits - i - 1] = '0' + (index % 10);
      index /= 10;
    }

    return x;
  };

  Update();

  renderer_->Render();

  // TODO: save screen shot
  /*
  std::string filename = ZeroPeddingIndex(4, dataset_->CurrentFrameIndex());
  std::replace(filename.begin(), filename.end(), '\\', '-');
  std::cout << "Saving to " << filename << "*.* ... ";
  renderer_->SaveImages(directory, "video", filename);
  std::cout << "Finished!" << std::endl;
  */
}

void Engine::SaveVideo()
{
  namespace fs = std::filesystem;

  std::string sequence_name = dataset_->GetCurrentSequenceName();
  std::replace(sequence_name.begin(), sequence_name.end(), '\\', '-');

  // TODO: linux directory character
  std::string directory = config_.GetVideoSaveDirectory() + '\\' + sequence_name;

  if (!fs::exists(directory))
  {
    std::cout << "Creating video directory: " << directory << std::endl;
    fs::create_directories(directory);
  }

  std::cout << "Saving image sequence file in: " << directory << std::endl;

  dataset_->SelectFrame(0);
  int frame = 0;

  auto ZeroPeddingIndex = [](int digits, int index) {
    std::string x(digits, '0');

    for (int i = 0; i < digits; i++)
    {
      x[digits - i - 1] = '0' + (index % 10);
      index /= 10;
    }

    return x;
  };

  animation_ = false;
  animation_time_ = 0.;
  animation_start_time_ = glfwGetTime();

  double video_length = dataset_->CurrentSequenceLength();
  double frame_length = 1. / dataset_->FrameRate();

  // Wnp has 60fps 
  if (dataset_->FrameRate() == 60)
    frame_length = 1. / 6.;

  while (true)
  {
    animation_time_ += frame_length;

    if (animation_time_ > video_length)
      break;

    Update();

    renderer_->Render();

    std::string filename = ZeroPeddingIndex(4, frame);
    std::replace(filename.begin(), filename.end(), '\\', '-');
    std::cout << "Saving to " << filename << "*.* ... ";
    renderer_->SaveImages(directory, "video", filename);
    std::cout << "Finished!" << std::endl;

    // Move to next frame until the end of frames
    frame++;
  }
}

void Engine::SaveCurrentTrajectory()
{
  std::cout << "Engine: Saving current trajectory" << std::endl;

  dataset_->SaveTrajectory(*trajectory_);
}

void Engine::Run()
{
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);


  // Renderer window
  window_renderer_ = glfwCreateWindow(width_, height_, "Iplanner", NULL, NULL);
  if (window_renderer_ == NULL)
  {
    glfwTerminate();
    throw std::runtime_error("Engine: Failed to create GLFW window.");
  }
  glfwSetWindowPos(window_renderer_, 10, 100);

  glfwMakeContextCurrent(window_renderer_);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    throw std::runtime_error("Engine: Failed to initialize GLAD.");


  // Callbacks
  glfwSetWindowUserPointer(window_renderer_, this);
  glfwSetMouseButtonCallback(window_renderer_, MouseButtonCallback);
  glfwSetCursorPosCallback(window_renderer_, CursorPosCallback);
  glfwSetKeyCallback(window_renderer_, KeyCallback);
  glfwSetFramebufferSizeCallback(window_renderer_, FramebufferSizeCallback);


  // Controller window
  window_controller_ = glfwCreateWindow(800, 800, "Iplanner Controller", NULL, NULL);
  if (window_controller_ == NULL)
  {
    glfwTerminate();
    throw std::runtime_error("Engine: Failed to create GLFW window.");
  }
  glfwSetWindowPos(window_controller_, 1000, 100);

  glfwMakeContextCurrent(window_controller_);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    throw std::runtime_error("Engine: Failed to initialize GLAD.");


  // Callbacks
  glfwSetWindowUserPointer(window_controller_, this);
  glfwSetMouseButtonCallback(window_controller_, MouseButtonCallback);
  glfwSetCursorPosCallback(window_controller_, CursorPosCallback);
  glfwSetKeyCallback(window_controller_, KeyCallback);
  glfwSetFramebufferSizeCallback(window_controller_, FramebufferSizeCallback);



  // Render
  Initialize();

  while (!glfwWindowShouldClose(window_renderer_) && !glfwWindowShouldClose(window_controller_))
  {
    using namespace std::chrono_literals;

    auto timestamp = std::chrono::system_clock::now();

    glfwPollEvents();


    // Draw renderer
    Update();

    if (redraw_)
    {
      glfwMakeContextCurrent(window_renderer_);
      renderer_->Render();

      glfwSwapBuffers(window_renderer_);

      redraw_ = false;
    }


    // Draw controller
    if (redraw_controller_)
    {
      glfwMakeContextCurrent(window_controller_);
      controller_->Render();

      glfwSwapBuffers(window_controller_);

      redraw_controller_ = false;
    }


    // Let CPU be idle for a short time window
    std::this_thread::sleep_until(timestamp + (1s / 120));
  }

  glfwTerminate();

  // Save current trajectory after exiting windows
  SaveCurrentTrajectory();

  planner_->Stop();
}

void Engine::Initialize()
{
  // Trajectory memory allocation
  trajectory_ = std::make_unique<Trajectory>(1, 2, 1.);

  // Create sample sequence list
  sample_sequences_ = config_.GetImageSamples();

  // Renderer
  glfwMakeContextCurrent(window_renderer_);

  glfwWindowHint(GLFW_SAMPLES, 4);
  glEnable(GL_MULTISAMPLE);

  renderer_ = std::make_unique<Renderer>(this);
  renderer_->SetColorCameraResolution(640, 480);
  renderer_->SetDepthCameraResolution(640, 480);
  renderer_->Initialize();


  // Controller
  glfwMakeContextCurrent(window_controller_);

  // TODO: hard coded window size
  controller_ = std::make_unique<Controller>(this, 800, 800);
  controller_->Initialize();


  // Renderer
  glfwMakeContextCurrent(window_renderer_);

  constexpr double pi = 3.1415926535897932384626433832795;
  view_camera_ = std::make_shared<Camera>();
  view_camera_->SetAspect(1280.f / 720.f);
  view_camera_->SetFovy(60.f / 180.f * pi);
  view_camera_->SetFar(10.f);

  // Kinect V2 has color image resolution of 1920 x 1080 pixels and a fov of 84.1 x 53.8 resulting in an average of about 22 x 20 pixels per degree.
  // https://smeenk.com/kinect-field-of-view-comparison/
  color_camera_ = std::make_shared<Camera>();
  color_camera_->SetAspect(640.f / 480.f);
  color_camera_->SetFovy(53.8f / 180.f * pi);
  color_camera_->SetFar(10.f);

  // Kinect V2 has a depth image resolution of 512 x 424 pixels with a fov of 70.6 x 60 degrees resulting in an average of about 7 x 7 pixels per degree
  depth_camera_ = std::make_shared<Camera>();
  depth_camera_->SetAspect(640.f / 480.f);
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


  // Full robot model
  /*
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

  // Robot planning model:
  /*
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

  // Dataset
  dataset_wnp_ = std::make_shared<Wnp>(config_.GetDatasetDirectory("wnp"));
  dataset_utkinect_ = std::make_shared<UtKinect>(config_.GetDatasetDirectory("utkinect"));
  dataset_occlusion_ = std::make_shared<DatasetOcclusion>(config_.GetDatasetDirectory("occlusion"));

  dataset_ = dataset_occlusion_;

  // Camera
  kinect_v1_ = std::make_shared<KinectV1>();
  kinect_v2_ = std::make_shared<KinectV2>();
  fake_kinect_ = std::make_shared<FakeKinect>();

  rgbd_camera_ = fake_kinect_;

  point_cloud_ = std::make_shared<PointCloud>();

  renderer_->CreateEmptyTexture("data_color", 640, 480, Texture::Usage::TEXTURE);
  renderer_->CreateEmptyTexture("data_depth", 640, 480, Texture::Usage::U16_TEXTURE); // data has column-major depth texture

  // Human
  human_model_ = dataset_->GetHumanModel();

  // Scene
  InitializeScene();

  // Planner
  planner_ = std::make_shared<Planner>();
  planner_->SetRobotModel(robot_arm_model_);
  planner_->Config().num_timepoints = 6;
  planner_->Config().prediction_timestep = 0.5;
  planner_->Config().time = 5.;

  planner_->RunAsync();

  ChooseOcclusion();


  // Set sequence is changed so sequence-associated initializations can be executed
  sequence_changed_ = true;
}

void Engine::InitializeScene()
{
  // Robot scene
  robot_node_ = RobotNode::Create(robot_arm_model_);
  auto root = scene_->GetRootNode();
  SceneNode::Connect(root, robot_node_);

  for (int i = 0; i < num_future_robots_; i++)
  {
    auto robot_node = RobotNode::Create(robot_arm_model_);
    robot_future_nodes_.push_back(robot_node);
    SceneNode::Connect(root, robot_node);
  }

  auto ground = std::make_shared<GroundNode>();
  SceneNode::Connect(root, ground);


  // Point cloud
  point_cloud_node_ = std::make_shared<PointCloudNode>();
  point_cloud_node_->SetPointCloud(point_cloud_);
  SceneNode::Connect(root, point_cloud_node_);


  // Human label scene
  human_label_node_kinect_v1_ = std::make_shared<HumanLabelNode>();
  human_label_node_kinect_v2_ = std::make_shared<HumanLabelNode>();
  human_label_node_fake_kinect_ = std::make_shared<HumanLabelNode>();

  // Kinect V1 human model edges
  human_label_node_kinect_v1_->AddEdge(0, 1);
  human_label_node_kinect_v1_->AddEdge(1, 2);
  human_label_node_kinect_v1_->AddEdge(2, 3);
  human_label_node_kinect_v1_->AddEdge(2, 3);
  human_label_node_kinect_v1_->AddEdge(2, 4);
  human_label_node_kinect_v1_->AddEdge(4, 5);
  human_label_node_kinect_v1_->AddEdge(5, 6);
  human_label_node_kinect_v1_->AddEdge(6, 7);
  human_label_node_kinect_v1_->AddEdge(2, 8);
  human_label_node_kinect_v1_->AddEdge(8, 9);
  human_label_node_kinect_v1_->AddEdge(9, 10);
  human_label_node_kinect_v1_->AddEdge(10, 11);
  human_label_node_kinect_v1_->AddEdge(0, 12);
  human_label_node_kinect_v1_->AddEdge(12, 13);
  human_label_node_kinect_v1_->AddEdge(13, 14);
  human_label_node_kinect_v1_->AddEdge(14, 15);
  human_label_node_kinect_v1_->AddEdge(0, 16);
  human_label_node_kinect_v1_->AddEdge(16, 17);
  human_label_node_kinect_v1_->AddEdge(17, 18);
  human_label_node_kinect_v1_->AddEdge(18, 19);

  // Kinect V2: additional supports near spine
  human_label_node_kinect_v1_->AddEdge(4, 12);
  human_label_node_kinect_v1_->AddEdge(8, 16);

  // Kinect V2 human model edges
  human_label_node_kinect_v2_->AddEdge(0, 1);
  human_label_node_kinect_v2_->AddEdge(1, 20);
  human_label_node_kinect_v2_->AddEdge(20, 2);
  human_label_node_kinect_v2_->AddEdge(2, 3);
  human_label_node_kinect_v2_->AddEdge(20, 4);
  human_label_node_kinect_v2_->AddEdge(4, 5);
  human_label_node_kinect_v2_->AddEdge(5, 6);
  human_label_node_kinect_v2_->AddEdge(6, 7);
  human_label_node_kinect_v2_->AddEdge(7, 21);
  human_label_node_kinect_v2_->AddEdge(6, 22);
  human_label_node_kinect_v2_->AddEdge(20, 8);
  human_label_node_kinect_v2_->AddEdge(8, 9);
  human_label_node_kinect_v2_->AddEdge(9, 10);
  human_label_node_kinect_v2_->AddEdge(10, 11);
  human_label_node_kinect_v2_->AddEdge(11, 23);
  human_label_node_kinect_v2_->AddEdge(10, 24);
  human_label_node_kinect_v2_->AddEdge(0, 12);
  human_label_node_kinect_v2_->AddEdge(12, 13);
  human_label_node_kinect_v2_->AddEdge(13, 14);
  human_label_node_kinect_v2_->AddEdge(14, 15);
  human_label_node_kinect_v2_->AddEdge(0, 16);
  human_label_node_kinect_v2_->AddEdge(16, 17);
  human_label_node_kinect_v2_->AddEdge(17, 18);
  human_label_node_kinect_v2_->AddEdge(18, 19);

  // Kinect V2: additional supports near spine
  human_label_node_kinect_v2_->AddEdge(1, 4);
  human_label_node_kinect_v2_->AddEdge(1, 8);
  human_label_node_kinect_v2_->AddEdge(1, 12);
  human_label_node_kinect_v2_->AddEdge(1, 16);

  // DatasetOcclusion human model edges
  human_label_node_fake_kinect_->AddEdge(8, 7);
  human_label_node_fake_kinect_->AddEdge(7, 0);
  human_label_node_fake_kinect_->AddEdge(7, 1);
  human_label_node_fake_kinect_->AddEdge(1, 2);
  human_label_node_fake_kinect_->AddEdge(2, 3);
  human_label_node_fake_kinect_->AddEdge(7, 4);
  human_label_node_fake_kinect_->AddEdge(4, 5);
  human_label_node_fake_kinect_->AddEdge(5, 6);
  human_label_node_fake_kinect_->AddEdge(8, 9);
  human_label_node_fake_kinect_->AddEdge(9, 10);
  human_label_node_fake_kinect_->AddEdge(10, 11);
  human_label_node_fake_kinect_->AddEdge(8, 12);
  human_label_node_fake_kinect_->AddEdge(12, 13);
  human_label_node_fake_kinect_->AddEdge(13, 14);

  // DatasetOcclusion: additional supports near spine
  human_label_node_fake_kinect_->AddEdge(1, 9);
  human_label_node_fake_kinect_->AddEdge(4, 12);

  SceneNode::Connect(root, human_label_node_kinect_v1_);
  SceneNode::Connect(root, human_label_node_kinect_v2_);
  SceneNode::Connect(root, human_label_node_fake_kinect_);

  human_label_node_ = human_label_node_fake_kinect_;
}

void Engine::Update()
{
  glfwMakeContextCurrent(window_renderer_);

  auto& state = *robot_state_;

  double t = glfwGetTime() - animation_start_time_;

  if (dataset_changed_)
  {
    Vector2i color_resolution{ dataset_->RgbWidth(), dataset_->RgbHeight() };
    Vector2i depth_resolution{ dataset_->DepthWidth(), dataset_->DepthHeight() };

    color_camera_->SetAspect(static_cast<float>(color_resolution(0)) / color_resolution(1));
    depth_camera_->SetAspect(static_cast<float>(depth_resolution(0)) / depth_resolution(1));

    renderer_->SetColorCameraResolution(color_resolution(0), color_resolution(1));
    renderer_->SetDepthCameraResolution(depth_resolution(0), depth_resolution(1));

    renderer_->ResizeTexture("data_color", color_resolution(0), color_resolution(1));
    renderer_->ResizeTexture("data_depth", depth_resolution(0), depth_resolution(1));

    dataset_changed_ = false;
  }

  if (sequence_changed_)
  {
    *trajectory_ = dataset_->GetTrajectory();
    const auto& trajectory = *trajectory_;

    // TODO: ignore first 3 joints to head camera
    constexpr int offset = 3;
    controller_->SetControllerSize(trajectory.Rows() - offset, trajectory.Cols());

    for (int i = offset; i < trajectory.Rows(); i++)
    {
      const auto& joint = robot_arm_model_->Joint(i);
      auto limit = joint.GetLimit();

      // TODO: continuous joints has no limits. The limits are set to [-PI, PI]
      if (limit.lower == 0. && limit.upper == 0.)
      {
        constexpr double pi = 3.1415926535897932384626433832795;
        limit.lower = -pi / 2.;
        limit.upper = pi / 2.;
      }

      for (int j = 0; j < trajectory.Cols(); j++)
      {
        auto value = trajectory(i, j);
        if (value < limit.lower)
          value = limit.lower;
        if (value > limit.upper)
          value = limit.upper;

        double controller_value = (value - limit.lower) / (limit.upper - limit.lower);

        controller_->At(i - offset, j) = controller_value;
      }
    }

    sequence_changed_ = false;
  }

  if (animation_)
    animation_time_ = t;

  auto frame_number = static_cast<int>(animation_time_ * dataset_->FrameRate());
  dataset_->SelectFrame(frame_number);

  // Update trajectory changed by controller
  // TODO: ignore first 3 joints to head camera
  constexpr int offset = 3;
  for (int i = offset; i < trajectory_->Rows(); i++)
  {
    const auto& joint = robot_arm_model_->Joint(i);
    auto limit = joint.GetLimit();

    // TODO: continuous joints has no limits. The limits are set to [-PI, PI]
    if (limit.lower == 0. && limit.upper == 0.)
    {
      constexpr double pi = 3.1415926535897932384626433832795;
      limit.lower = -pi / 2.;
      limit.upper = pi / 2.;
    }

    for (int j = 0; j < trajectory_->Cols(); j++)
    {
      double controller_value = controller_->At(i - offset, j);
      double joint_value = limit.lower + controller_value * (limit.upper - limit.lower);
      (*trajectory_)(i, j) = joint_value;
    }
  }

  // Update point cloud
  rgbd_camera_->FeedFrame(dataset_->GetRgbImage(), dataset_->GetDepthImage());
  rgbd_camera_->GeneratePointCloud();
  rgbd_camera_->GetPointCloud(point_cloud_);
  point_cloud_node_->UpdateBuffer();

  // Update color/depth images from Kinect sensor
  renderer_->UpdateTexture("data_color", rgbd_camera_->GetColorBuffer());
  renderer_->UpdateTexture("data_depth", rgbd_camera_->GetDepthBuffer());

  // Update human model
  human_label_node_->SetLabel(dataset_->GetHumanLabel());

  // sin function motion
  /*
  double v = (std::sin(5. * animation_time_) + 1.) / 2. * 0.2;
  state.JointPosition(4) = -0.3 + v;
  */

  // Robot trajectory from planner
  /*
  auto trajectory_point = planner_->GetTrajectory().AtTime(animation_time_);
  for (int i = 0; i < state.NumJoints(); i++)
    state.JointPosition(i) = trajectory_point(i);
    */

  // Robot trajectory from dataset
  /*
  auto trajectory = dataset_->GetTrajectory();
  std::cout << "trajectory dimension: " << trajectory.Rows() << " x " << trajectory.Cols() << std::endl;
  auto trajectory_point = trajectory.AtTime(animation_time_);
  */

  // Robot trajectory from controller
  VectorXd trajectory_point;

  if (animation_)
    trajectory_point = trajectory_->AtTime(animation_time_);
  else
    trajectory_point = trajectory_->AtTime(dataset_->CurrentTime());

  for (int i = 0; i < state.NumJoints() && i < trajectory_->Rows(); i++)
    state.JointPosition(i) = trajectory_point(i);

  for (int i = trajectory_->Rows(); i < state.NumJoints(); i++)
    state.JointPosition(i) = 0.;

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

  // TODO: Future robot nodes
  for (int i = 0; i < num_future_robots_; i++)
  {
    const double future_time = future_robot_timestep_ * (i + 1);

    // Robot trajectory from controller
    VectorXd trajectory_point;

    if (animation_)
      trajectory_point = trajectory_->AtTime(animation_time_ + future_time);
    else
      trajectory_point = trajectory_->AtTime(dataset_->CurrentTime() + future_time);

    for (int j = 0; j < state.NumJoints(); j++)
      robot_future_nodes_[i]->SetJointValue(state.JointName(j), trajectory_point(j));
  }

  redraw_ = true;

  // TODO: move controller redraw to where the controller state changesn
  redraw_controller_ = true;
}

Vector2i Engine::GetScreenSize() const
{
  return Vector2i(width_, height_);
}
}
