#version 430

in vec2 vertex_tex_coord;

uniform sampler2D tex;

out vec4 out_color;

void main()
{
  out_color = vec4(texture(tex, vertex_tex_coord).rgb, 1.f);
}
