#version 430

layout (location = 0) in vec3 position;
layout (location = 1) in vec2 tex_coord;

out vec3 vertex_position;
out vec2 vertex_tex_coord;

void main()
{
  gl_Position = vec4(position, 1.f);
  vertex_position = position;
  vertex_tex_coord = tex_coord;
}
