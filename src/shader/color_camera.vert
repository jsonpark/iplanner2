#version 430

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 tex_coord;

layout (binding = 0, std140) uniform Camera
{
  mat4 projection;
  mat4 view;
};

uniform mat4 model;

out vec3 vertex_position;
out vec2 vertex_tex_coord;

void main()
{
  gl_Position = projection * view * model * vec4(position, 1.f);
  vertex_position = vec3(model * vec4(position, 1.f));
  vertex_tex_coord = tex_coord;
}
