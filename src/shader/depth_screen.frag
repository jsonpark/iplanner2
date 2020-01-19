#version 430

in vec2 vertex_tex_coord;

uniform usampler2D depth;

uniform vec2 nearfar;

out vec4 out_color;

void main()
{
  uint du = texture(depth, vertex_tex_coord).r;

  float d = (float(du) / 1000.f) / 8.f;
  out_color = vec4(vec3(d), 1.f);
}
