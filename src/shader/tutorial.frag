#version 430

in vec3 vertex_color;
in vec2 vertex_tex_coord;

uniform sampler2D tex;

out vec4 out_color;

void main()
{
  out_color = vec4(texture(tex, vertex_tex_coord).rgb * vertex_color, 1.f);
}
