#version 430

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 color;

layout (binding = 0, std140) uniform Camera
{
  uniform mat4 projection;
  uniform mat4 view;
};

uniform mat4 model;

uniform vec4 width_height_near_far;
uniform float point_size;

out Vertex
{
  vec3 color;
  float width;
  float height;
} frag;

void main()
{
  vec4 vm = view * model * vec4(position, 1.f);
  gl_Position = projection * vm;

  frag.color = color;
  frag.width = (-vm.z * 2.f / width_height_near_far[0]) * point_size;
  frag.height = (-vm.z * 2.f / width_height_near_far[1]) * point_size;
}
