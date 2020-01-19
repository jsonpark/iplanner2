#version 430

in vec3 vertex_position;
in vec2 vertex_tex_coord;

uniform sampler2D tex;

layout (location = 0) out vec4 out_color;
layout (location = 1) out uint out_depth;

void main()
{
  out_color = vec4(texture(tex, vertex_tex_coord).rgb, 1.f);
  out_depth = uint((vertex_position.z + 1.f) * 1000.);
}
