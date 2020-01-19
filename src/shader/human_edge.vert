#version 430

layout (location = 0) in vec3 position;

layout (binding = 0, std140) uniform Camera
{
  uniform mat4 projection;
  uniform mat4 view;
};

uniform mat4 model;

void main()
{
  vec4 vm = view * model * vec4(position, 1.f);
  gl_Position = projection * vm;
}
