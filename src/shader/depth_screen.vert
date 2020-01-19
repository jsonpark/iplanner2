#version 430

layout (location = 0) in vec2 position;
layout (location = 1) in vec2 tex_coord;

out vec2 vertex_tex_coord;

void main()
{
  gl_Position = vec4(position, 0.f, 1.f);
  vertex_tex_coord = tex_coord;
}
