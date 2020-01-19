#version 430

layout (location = 0) in vec3 position;
layout (location = 2) in vec2 tex_coord;

layout (binding = 0, std140) uniform Camera
{
  mat4 projection;
  mat4 view;
};

uniform mat4 model;

out vec4 vertex_position;

void main()
{
  gl_Position = projection * view * model * vec4(position, 1.f);
  vertex_position = vec4(view * model * vec4(position, 1.f));
}
